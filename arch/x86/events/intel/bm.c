/*
 * Support for Intel branch monitoring counters
 *
 * Intel branch monitoring MSRs are specified in the Intel® 64 and IA-32
 * Software Developer’s Manual Volume 4 section 2.16.2 (October 2017)
 *
 * Copyright (c) 2017, Intel Corporation.
 *
 * Contact Information:
 * Megha Dey <megha.dey@linux.intel.com>
 * Yu-Cheng Yu <yu-cheng.yu@intel.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 */

#include <linux/module.h>
#include <linux/perf_event.h>
#include <linux/slab.h>
#include <linux/poll.h>
#include <linux/err.h>
#include <asm/apic.h>
#include <asm/cpu_device_id.h>
#include <asm/intel-family.h>
#include <asm/nmi.h>

#include "../perf_event.h"

/* Branch Monitoring default and mask values */
#define BM_MAX_WINDOW_SIZE		0x3ff
#define BM_MAX_THRESHOLD		0x7f
#define BM_MAX_EVENTS			6
#define BM_WINDOW_SIZE_SHIFT		8
#define BM_THRESHOLD_SHIFT		8
#define BM_EVENT_TYPE_SHIFT		1
#define BM_GUEST_DISABLE_SHIFT		3
#define BM_LBR_FREEZE_SHIFT		2
#define BM_WINDOW_CNT_SEL_SHIFT		24
#define BM_CNT_AND_MODE_SHIFT		26
#define BM_MISPRED_EVT_CNT_SHIFT	15
#define BM_ENABLE			0x3

static unsigned int bm_window_size = BM_MAX_WINDOW_SIZE;
static unsigned int bm_guest_disable;
static unsigned int bm_lbr_freeze;
static unsigned int bm_window_cnt_sel;
static unsigned int bm_cnt_and_mode;

static unsigned int bm_threshold = BM_MAX_THRESHOLD;
static unsigned int bm_mispred_evt_cnt;

/* Branch monitoring counter owners */
static struct perf_event *bm_counter_owner[2];

static struct pmu intel_bm_pmu;

static int bm_num_counters;

DEFINE_PER_CPU(int, bm_unmask_apic);

union bm_detect_status {
	struct {
		__u8 event: 1;
		__u8 lbrs_valid: 1;
		__u8 reserved0: 6;
		__u8 ctrl_hit: 4;
		__u8 reserved1: 4;
		__u16 count_window: 10;
		__u8 reserved2: 6;
		__u8 count[4];
	} __packed;
	uint64_t raw;
};

static int intel_bm_event_nmi_handler(unsigned int cmd, struct pt_regs *regs)
{
	struct perf_event *event;
	union bm_detect_status stat;
	struct perf_sample_data data;
	int i;
	unsigned long x;

	rdmsrl(BR_DETECT_STATUS_MSR, stat.raw);

	if (stat.event) {
		wrmsrl(BR_DETECT_STATUS_MSR, 0);
		apic_write(APIC_LVTPC, APIC_DM_NMI);
		/*
		 * Issue wake-up to corrresponding polling event
		 */
		x = stat.ctrl_hit;
		for_each_set_bit(i, &x, bm_num_counters) {
			event = bm_counter_owner[i];
			perf_sample_data_init(&data, 0, event->hw.last_period);
			perf_event_overflow(event, &data, regs);
			local64_inc(&event->count);
			atomic_set(&event->hw.bm_poll, POLLIN);
			event->pending_wakeup = 1;
			irq_work_queue(&event->pending);
		}
		return NMI_HANDLED;
	}
	return NMI_DONE;
}

static void intel_bm_event_start(struct perf_event *event, int mode)
{
	WARN_ON(event->id >= bm_num_counters);

	wrmsrl(BR_DETECT_COUNTER_CONFIG_BASE + event->id,
	       (event->hw.bm_counter_conf | 1));
}

/*
 * Unmask the NMI bit of the local APIC the first time task is scheduled
 * on a particular CPU.
 */
static void intel_bm_unmask_nmi(void)
{
	this_cpu_write(bm_unmask_apic, 0);

	if (!(this_cpu_read(bm_unmask_apic))) {
		apic_write(APIC_LVTPC, APIC_DM_NMI);
		this_cpu_inc(bm_unmask_apic);
	}
}

static int intel_bm_event_add(struct perf_event *event, int mode)
{
	union bm_detect_status cur_stat, prev_stat;

	prev_stat.raw = local64_read(&event->hw.prev_count);

	/*
	 * Start counting from previous count associated with this event
	 */
	rdmsrl(BR_DETECT_STATUS_MSR, cur_stat.raw);

	cur_stat.count[event->id] = prev_stat.count[event->id];
	cur_stat.count_window = prev_stat.count_window;
	wrmsrl(BR_DETECT_STATUS_MSR, cur_stat.raw);

	wrmsrl(BR_DETECT_CONTROL_MSR, event->hw.bm_ctrl);

	intel_bm_unmask_nmi();

	intel_bm_event_start(event, mode);

	return 0;
}

static void intel_bm_event_update(struct perf_event *event)
{
	union bm_detect_status cur_stat;

	rdmsrl(BR_DETECT_STATUS_MSR, cur_stat.raw);
	local64_set(&event->hw.prev_count, (uint64_t)cur_stat.raw);
}

static void intel_bm_event_stop(struct perf_event *event, int mode)
{
	WARN_ON(event->id >= bm_num_counters);

	wrmsrl(BR_DETECT_COUNTER_CONFIG_BASE + event->id,
	       (event->hw.bm_counter_conf & ~1));

	intel_bm_event_update(event);
}

static void intel_bm_event_del(struct perf_event *event, int flags)
{
	intel_bm_event_stop(event, flags);
}

static void intel_bm_event_read(struct perf_event *event)
{
}

static void intel_bm_event_destroy(struct perf_event *event)
{
	bm_counter_owner[event->id] = NULL;
}

static int intel_bm_event_init(struct perf_event *event)
{
	u64 cfg;
	int counter_to_use = -1, i;

	local64_set(&event->hw.prev_count, 0);

	if (perf_paranoid_cpu() && !capable(CAP_SYS_ADMIN))
		return -EACCES;

	/*
	 * Type is assigned by kernel, see /sys/devices/intel_bm/type
	 */
	if (event->attr.type != intel_bm_pmu.type)
		return -ENOENT;

	/*
	 * Only per tasks events are supported. It does not make sense to
	 * monitor all tasks for an ROP attack. This could generate a lot
	 * of false positives.
	 */
	if (event->hw.target == NULL)
		return -EINVAL;

	event->event_caps |= PERF_EV_CAP_BM;
	/*
	 * cfg contains one of the 6 possible Branch Monitoring events
	 */
	cfg = event->attr.config;
	if (cfg < 0 || cfg > (BM_MAX_EVENTS - 1))
		return -EINVAL;

	if (event->attr.sample_period) /* no sampling */
		return -EINVAL;

	/*
	 * Find a hardware counter for the target task
	 */
	for (i = 0; i < bm_num_counters; i++) {
		if ((bm_counter_owner[i] == NULL) ||
			(bm_counter_owner[i]->state == PERF_EVENT_STATE_DEAD)) {
			counter_to_use = i;
			bm_counter_owner[i] = event;
			break;
		}
	}

	if (counter_to_use == -1)
		return -EBUSY;

	event->hw.bm_ctrl = (bm_window_size << BM_WINDOW_SIZE_SHIFT) |
			    (bm_guest_disable << BM_GUEST_DISABLE_SHIFT) |
			    (bm_lbr_freeze << BM_LBR_FREEZE_SHIFT) |
			    (bm_window_cnt_sel << BM_WINDOW_CNT_SEL_SHIFT) |
			    (bm_cnt_and_mode << BM_CNT_AND_MODE_SHIFT) |
								BM_ENABLE;
	event->hw.bm_counter_conf = (bm_threshold << BM_THRESHOLD_SHIFT) |
			(bm_mispred_evt_cnt << BM_MISPRED_EVT_CNT_SHIFT) |
					(cfg << BM_EVENT_TYPE_SHIFT);

	wrmsrl(BR_DETECT_COUNTER_CONFIG_BASE + counter_to_use,
						event->hw.bm_counter_conf);
	wrmsrl(BR_DETECT_STATUS_MSR, 0);
	event->id = counter_to_use;
	local64_set(&event->count, 0);

	event->destroy = intel_bm_event_destroy;

	return 0;
}

EVENT_ATTR_STR(rets, rets, "event=0x0");
EVENT_ATTR_STR(call-ret, call_ret, "event=0x01");
EVENT_ATTR_STR(ret-misp, ret_misp, "event=0x02");
EVENT_ATTR_STR(branch-misp, branch_mispredict, "event=0x03");
EVENT_ATTR_STR(indirect-branch-misp, indirect_branch_mispredict, "event=0x04");
EVENT_ATTR_STR(far-branch, far_branch, "event=0x05");

static struct attribute *intel_bm_events_attr[] = {
	EVENT_PTR(rets),
	EVENT_PTR(call_ret),
	EVENT_PTR(ret_misp),
	EVENT_PTR(branch_mispredict),
	EVENT_PTR(indirect_branch_mispredict),
	EVENT_PTR(far_branch),
	NULL,
};

static struct attribute_group intel_bm_events_group = {
	.name = "events",
	.attrs = intel_bm_events_attr,
};

PMU_FORMAT_ATTR(event, "config:0-7");
static struct attribute *intel_bm_formats_attr[] = {
	&format_attr_event.attr,
	NULL,
};

static struct attribute_group intel_bm_format_group = {
	.name = "format",
	.attrs = intel_bm_formats_attr,
};

/*
 * User can configure the BM MSRs using the corresponding sysfs entries
 */

static ssize_t
threshold_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	ssize_t rv;

	rv = sprintf(buf, "%d\n", bm_threshold);

	return rv;
}

static ssize_t
threshold_store(struct device *dev,
			   struct device_attribute *attr,
			   const char *buf, size_t count)
{
	unsigned int threshold;
	int err;

	err = kstrtouint(buf, 0, &threshold);
	if (err)
		return err;

	if (threshold > BM_MAX_THRESHOLD) {
		pr_err("invalid threshold value\n");
		return -EINVAL;
	}

	bm_threshold = threshold;

	return count;
}

static DEVICE_ATTR_RW(threshold);

static ssize_t
window_size_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	ssize_t rv;

	rv = sprintf(buf, "%d\n", bm_window_size);

	return rv;
}

static ssize_t
window_size_store(struct device *dev,
			    struct device_attribute *attr,
			    const char *buf, size_t count)
{
	unsigned int window_size;
	int err;

	err = kstrtouint(buf, 0, &window_size);
	if (err)
		return err;

	if (window_size > BM_MAX_WINDOW_SIZE) {
		pr_err("illegal window size\n");
		return -EINVAL;
	}

	bm_window_size = window_size;

	return count;
}

static DEVICE_ATTR_RW(window_size);

static ssize_t
lbr_freeze_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	ssize_t rv;

	rv = sprintf(buf, "%d\n", bm_lbr_freeze);

	return rv;
}

static ssize_t
lbr_freeze_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	unsigned int lbr_freeze;
	int err;

	err = kstrtouint(buf, 0, &lbr_freeze);
	if (err)
		return err;

	if (lbr_freeze > 1) {
		pr_err("lbr freeze can only be 0 or 1\n");
		return -EINVAL;
	}

	bm_lbr_freeze = lbr_freeze;

	return count;
}

static DEVICE_ATTR_RW(lbr_freeze);

static ssize_t
guest_disable_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	ssize_t rv;

	rv = sprintf(buf, "%d\n", bm_guest_disable);

	return rv;
}

static ssize_t
guest_disable_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	unsigned int guest_disable;
	int err;

	err = kstrtouint(buf, 0, &guest_disable);
	if (err)
		return err;

	if (guest_disable > 1) {
		pr_err("guest disable can only be 0 or 1\n");
		return -EINVAL;
	}

	bm_guest_disable = guest_disable;

	return count;
}

static DEVICE_ATTR_RW(guest_disable);

static ssize_t
window_cnt_sel_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	ssize_t rv;

	rv = sprintf(buf, "%d\n", bm_window_cnt_sel);

	return rv;
}

static ssize_t
window_cnt_sel_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	unsigned int window_cnt_sel;
	int err;

	err = kstrtouint(buf, 0, &window_cnt_sel);
	if (err)
		return err;

	if (window_cnt_sel > 3) {
		pr_err("invalid window_cnt_sel value\n");
		return -EINVAL;
	}

	bm_window_cnt_sel = window_cnt_sel;

	return count;
}

static DEVICE_ATTR_RW(window_cnt_sel);

static ssize_t
cnt_and_mode_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	ssize_t rv;

	rv = sprintf(buf, "%d\n", bm_cnt_and_mode);

	return rv;
}

static ssize_t
cnt_and_mode_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	unsigned int cnt_and_mode;
	int err;

	err = kstrtouint(buf, 0, &cnt_and_mode);
	if (err)
		return err;

	if (cnt_and_mode > 1) {
		pr_err("invalid cnt_and_mode value\n");
		return -EINVAL;
	}

	bm_cnt_and_mode = cnt_and_mode;

	return count;
}

static DEVICE_ATTR_RW(cnt_and_mode);

static ssize_t
mispred_evt_cnt_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	ssize_t rv;

	rv = sprintf(buf, "%d\n", bm_mispred_evt_cnt);

	return rv;
}

static ssize_t
mispred_evt_cnt_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	unsigned int mispred_evt_cnt;
	int err;

	err = kstrtouint(buf, 0, &mispred_evt_cnt);
	if (err)
		return err;

	if (mispred_evt_cnt > 1) {
		pr_err("invalid mispred_evt_cnt value\n");
		return -EINVAL;
	}

	bm_mispred_evt_cnt = mispred_evt_cnt;

	return count;
}

static DEVICE_ATTR_RW(mispred_evt_cnt);

static ssize_t
num_counters_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	ssize_t rv;

	rv = sprintf(buf, "%d\n", bm_num_counters);

	return rv;
}

static DEVICE_ATTR_RO(num_counters);

static struct attribute *intel_bm_attrs[] = {
	&dev_attr_window_size.attr,
	&dev_attr_threshold.attr,
	&dev_attr_lbr_freeze.attr,
	&dev_attr_guest_disable.attr,
	&dev_attr_window_cnt_sel.attr,
	&dev_attr_cnt_and_mode.attr,
	&dev_attr_mispred_evt_cnt.attr,
	&dev_attr_num_counters.attr,
	NULL,
};

static const struct attribute_group intel_bm_group = {
	.attrs = intel_bm_attrs,
};

static const struct attribute_group *intel_bm_attr_groups[] = {
	&intel_bm_events_group,
	&intel_bm_format_group,
	&intel_bm_group,
	NULL,
};

static struct pmu intel_bm_pmu = {
	.task_ctx_nr     = perf_sw_context,
	.attr_groups     = intel_bm_attr_groups,
	.event_init      = intel_bm_event_init,
	.add             = intel_bm_event_add,
	.del             = intel_bm_event_del,
	.start           = intel_bm_event_start,
	.stop            = intel_bm_event_stop,
	.read            = intel_bm_event_read,
};

#define X86_BM_MODEL_MATCH(model)       \
	{ X86_VENDOR_INTEL, 6, model, X86_FEATURE_ANY }

static const struct x86_cpu_id bm_cpu_match[] __initconst = {
	X86_BM_MODEL_MATCH(INTEL_FAM6_CANNONLAKE_MOBILE),
	{},
};

MODULE_DEVICE_TABLE(x86cpu, bm_cpu_match);

static __init int intel_bm_init(void)
{
	int ret, err;

	/*
	 * Only CNL supports branch monitoring
	 */
	if (!(x86_match_cpu(bm_cpu_match)))
		return -ENODEV;

	bm_num_counters = 2;

	err = register_nmi_handler(NMI_LOCAL, intel_bm_event_nmi_handler,
								0, "BM");

	if (err)
		goto fail_nmi;

	ret =  perf_pmu_register(&intel_bm_pmu, "intel_bm", -1);
	if (ret) {
		pr_err("Intel BM perf registration failed: %d\n", ret);
		return ret;
	}

	return 0;

fail_nmi:
	unregister_nmi_handler(NMI_LOCAL, "BM");
	return err;
}
module_init(intel_bm_init);

static void __exit intel_bm_exit(void)
{
	perf_pmu_unregister(&intel_bm_pmu);
	unregister_nmi_handler(NMI_LOCAL, "BM");
}
module_exit(intel_bm_exit);

MODULE_LICENSE("GPL");
