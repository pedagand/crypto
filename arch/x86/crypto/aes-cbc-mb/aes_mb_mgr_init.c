/*
 * Initialization code for multi buffer AES CBC algorithm
 *
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright(c) 2016 Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * Contact Information:
 * James Guilford <james.guilford@intel.com>
 * Sean Gulley <sean.m.gulley@intel.com>
 * Tim Chen <tim.c.chen@linux.intel.com>
 * Megha Dey <megha.dey@linux.intel.com>
 *
 * BSD LICENSE
 *
 * Copyright(c) 2016 Intel Corporation.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in
 * the documentation and/or other materials provided with the
 * distribution.
 * Neither the name of Intel Corporation nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "aes_cbc_mb_mgr.h"

void aes_cbc_init_mb_mgr_inorder_x8(struct aes_cbc_mb_mgr_inorder_x8 *state)
{
	/* Init "out of order" components */
	state->unused_lanes = 0xF76543210;
	state->job_in_lane[0] = NULL;
	state->job_in_lane[1] = NULL;
	state->job_in_lane[2] = NULL;
	state->job_in_lane[3] = NULL;
	state->job_in_lane[4] = NULL;
	state->job_in_lane[5] = NULL;
	state->job_in_lane[6] = NULL;
	state->job_in_lane[7] = NULL;

	/* Init "in order" components */
	state->next_job = 0;
	state->earliest_job = -1;

}

#define JOBS(offset) ((struct job_aes_cbc *)(((u64)state->jobs)+offset))

struct job_aes_cbc *
aes_cbc_get_next_job_inorder_x8(struct aes_cbc_mb_mgr_inorder_x8 *state)
{
	return JOBS(state->next_job);
}

struct job_aes_cbc *
aes_cbc_flush_job_inorder_x8(struct aes_cbc_mb_mgr_inorder_x8 *state)
{
	struct job_aes_cbc *job;

	/* checking earliest_job < 0 fails and the code walks over bogus */
	if (state->earliest_job == -1)
		return NULL; /* empty */

	job = JOBS(state->earliest_job);
	while (job->status != STS_COMPLETED) {
		switch (job->key_len) {
		case AES_KEYSIZE_128:
			aes_cbc_flush_job_ooo_128x8(state);
			break;
		case AES_KEYSIZE_192:
			aes_cbc_flush_job_ooo_192x8(state);
			break;
		case AES_KEYSIZE_256:
			aes_cbc_flush_job_ooo_256x8(state);
			break;
		default:
			break;
		}
	}

	/* advance earliest job */
	state->earliest_job += sizeof(struct job_aes_cbc);
	if (state->earliest_job == MAX_AES_JOBS * sizeof(struct job_aes_cbc))
		state->earliest_job = 0;

	if (state->earliest_job == state->next_job)
		state->earliest_job = -1;

	return job;
}

struct job_aes_cbc *
aes_cbc_get_completed_job_inorder_x8(struct aes_cbc_mb_mgr_inorder_x8 *state)
{
	struct job_aes_cbc *job;

	if (state->earliest_job == -1)
		return NULL; /* empty */

	job = JOBS(state->earliest_job);
	if (job->status != STS_COMPLETED)
		return NULL;

	state->earliest_job += sizeof(struct job_aes_cbc);

	if (state->earliest_job == MAX_AES_JOBS * sizeof(struct job_aes_cbc))
		state->earliest_job = 0;
	if (state->earliest_job == state->next_job)
		state->earliest_job = -1;

	/* we have a completed job */
	return job;
}
