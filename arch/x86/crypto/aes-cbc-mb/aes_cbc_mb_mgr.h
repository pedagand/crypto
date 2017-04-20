/*
 *	Header file for multi buffer AES CBC algorithm manager
 *	that deals with 8 buffers at a time
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
#ifndef __AES_CBC_MB_MGR_H
#define __AES_CBC_MB_MGR_H


#include <linux/types.h>
#include <linux/printk.h>
#include <crypto/aes.h>
#include <crypto/b128ops.h>

#define MAX_AES_JOBS		128

enum job_sts {
	STS_UNKNOWN = 0,
	STS_BEING_PROCESSED = 1,
	STS_COMPLETED = 2,
	STS_INTERNAL_ERROR = 3,
	STS_ERROR = 4
};

/* AES CBC multi buffer in order job structure */

struct job_aes_cbc {
	u8	*plaintext;	/* pointer to plaintext */
	u8	*ciphertext;	/* pointer to ciphertext */
	u128	iv;		/* initialization vector */
	u128	*keys;		/* pointer to keys */
	u32	len;		/* length in bytes, must be multiple of 16 */
	enum	job_sts status;	/* status enumeration */
	void	*user_data;	/* pointer to user data */
	u32	key_len;	/* key length */
};

struct aes_cbc_args_x8 {
	u8	*arg_in[8];	/* array of 8 pointers to in text */
	u8	*arg_out[8];	/* array of 8 pointers to out text */
	u128	*arg_keys[8];	/* array of 8 pointers to keys */
	u128	arg_iv[8] __aligned(16);	/* array of 8 128-bit IVs */
};

struct aes_cbc_mb_mgr_inorder_x8 {
	struct aes_cbc_args_x8 args;
	u16 lens[8] __aligned(16);
	u64 unused_lanes; /* each nibble is index (0...7) of unused lanes */
	/* nibble 8 is set to F as a flag */
	struct job_aes_cbc *job_in_lane[8];
	/* In-order components */
	u32 earliest_job; /* byte offset, -1 if none */
	u32 next_job;      /* byte offset */
	struct job_aes_cbc jobs[MAX_AES_JOBS];
};

/* define AES CBC multi buffer manager function proto */
struct job_aes_cbc *aes_cbc_submit_job_inorder_128x8(
	struct aes_cbc_mb_mgr_inorder_x8 *state);
struct job_aes_cbc *aes_cbc_submit_job_inorder_192x8(
	struct aes_cbc_mb_mgr_inorder_x8 *state);
struct job_aes_cbc *aes_cbc_submit_job_inorder_256x8(
	struct aes_cbc_mb_mgr_inorder_x8 *state);
struct job_aes_cbc *aes_cbc_flush_job_inorder_x8(
	struct aes_cbc_mb_mgr_inorder_x8 *state);
struct job_aes_cbc *aes_cbc_get_next_job_inorder_x8(
	struct aes_cbc_mb_mgr_inorder_x8 *state);
struct job_aes_cbc *aes_cbc_get_completed_job_inorder_x8(
	struct aes_cbc_mb_mgr_inorder_x8 *state);
void aes_cbc_init_mb_mgr_inorder_x8(
	struct aes_cbc_mb_mgr_inorder_x8 *state);
void aes_cbc_submit_job_ooo_x8(struct aes_cbc_mb_mgr_inorder_x8 *state,
		struct job_aes_cbc *job);
void aes_cbc_flush_job_ooo_x8(struct aes_cbc_mb_mgr_inorder_x8 *state);
void aes_cbc_flush_job_ooo_128x8(struct aes_cbc_mb_mgr_inorder_x8 *state);
void aes_cbc_flush_job_ooo_192x8(struct aes_cbc_mb_mgr_inorder_x8 *state);
void aes_cbc_flush_job_ooo_256x8(struct aes_cbc_mb_mgr_inorder_x8 *state);

#endif /* __AES_CBC_MB_MGR_H */
