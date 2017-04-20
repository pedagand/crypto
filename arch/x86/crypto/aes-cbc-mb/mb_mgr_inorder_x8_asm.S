/*
 *	AES CBC by8 multibuffer inorder scheduler optimization (x86_64)
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
#include <linux/linkage.h>
#include "mb_mgr_datastruct.S"
#include "reg_sizes.S"

#define JUMP

#define arg1	%rdi
#define arg2	%rsi
#define state	arg1

/* virtual registers used by submit_job_aes_inorder_x8 */
#define next_job	%rdx
#define earliest_job	%rcx
#define zero		%r8
#define returned_job	%rax	/* register that returns a value from func */

.extern aes_cbc_submit_job_ooo_128x8
.extern aes_cbc_submit_job_ooo_192x8
.extern aes_cbc_submit_job_ooo_256x8
.extern aes_cbc_flush_job_ooo_128x8
.extern aes_cbc_flush_job_ooo_192x8
.extern aes_cbc_flush_job_ooo_256x8
.extern aes_cbc_flush_job_ooo_x8

/*
 * struct job_aes* aes_cbc_submit_job_inorder_x8(
 *		struct aes_cbc_mb_mgr_inorder_x8 *state)
 */

.macro aes_cbc_submit_job_inorder_x8 key_len

	sub	$8, %rsp	/* align stack for next calls */

	mov	_next_job(state), DWORD(next_job)
	lea	_jobs(state, next_job), arg2

	.if \key_len == AES_KEYSIZE_128
		call	aes_cbc_submit_job_ooo_128x8
	.elseif \key_len == AES_KEYSIZE_192
		call	aes_cbc_submit_job_ooo_192x8
	.elseif \key_len == AES_KEYSIZE_256
		call	aes_cbc_submit_job_ooo_256x8
	.endif

	mov	_earliest_job(state), DWORD(earliest_job)
	cmp	$0, DWORD(earliest_job)
	jl	.Lstate_was_empty\key_len

	/* we have a valid earliest_job */

	/* advance next_job */
	mov	_next_job(state), DWORD(next_job)
	add	$_JOB_AES_size, next_job
#ifdef JUMP
	cmp	$(MAX_AES_JOBS * _JOB_AES_size), next_job
	jne	.Lskip1\key_len
	xor	next_job, next_job
.Lskip1\key_len:
#else
	xor	zero,zero
	cmp	$(MAX_AES_JOBS * _JOB_AES_size), next_job
	cmove	zero, next_job
#endif
	mov	DWORD(next_job), _next_job(state)

	lea	_jobs(state, earliest_job), returned_job
	cmp	next_job, earliest_job
	je	.Lfull\key_len

	/* not full */
	cmpl	$STS_COMPLETED, _status(returned_job)
	jne	.Lreturn_null\key_len

	/* advance earliest_job */
	add	$_JOB_AES_size, earliest_job
	cmp	$(MAX_AES_JOBS * _JOB_AES_size), earliest_job
#ifdef JUMP
	jne	.Lskip2\key_len
	xor	earliest_job, earliest_job
.Lskip2\key_len:
#else
	cmove	zero, earliest_job
#endif

	add	$8, %rsp
	mov	DWORD(earliest_job), _earliest_job(state)
	ret

.Lreturn_null\key_len:
	add	$8, %rsp
	xor	returned_job, returned_job
	ret

.Lfull\key_len:
	cmpl	$STS_COMPLETED, _status(returned_job)
	je	.Lcompleted\key_len
	mov	earliest_job, (%rsp)
.Lflush_loop\key_len:
	.if \key_len == AES_KEYSIZE_128
		call	aes_cbc_flush_job_ooo_128x8
	.elseif \key_len == AES_KEYSIZE_192
		call	aes_cbc_flush_job_ooo_192x8
	.elseif \key_len == AES_KEYSIZE_256
		call	aes_cbc_flush_job_ooo_256x8
	.endif
	/* state is still valid */
	mov	(%rsp), earliest_job
	cmpl	$STS_COMPLETED, _status(returned_job)
	jne	.Lflush_loop\key_len
	xor	zero,zero
.Lcompleted\key_len:
	/* advance earliest_job */
	add	$_JOB_AES_size, earliest_job
	cmp	$(MAX_AES_JOBS * _JOB_AES_size), earliest_job
#ifdef JUMP
	jne	.Lskip3\key_len
	xor	earliest_job, earliest_job
.Lskip3\key_len:
#else
	cmove	zero, earliest_job
#endif

	add	$8, %rsp
	mov	DWORD(earliest_job), _earliest_job(state)
	ret

.Lstate_was_empty\key_len:
	mov	_next_job(state), DWORD(next_job)
	mov	DWORD(next_job), _earliest_job(state)

	/* advance next_job */
	add	$_JOB_AES_size, next_job
#ifdef JUMP
	cmp	$(MAX_AES_JOBS * _JOB_AES_size), next_job
	jne	.Lskip4\key_len
	xor	next_job, next_job
.Lskip4\key_len:
#else
	xor	zero,zero
	cmp	$(MAX_AES_JOBS * _JOB_AES_size), next_job
	cmove	zero, next_job
#endif
	mov	DWORD(next_job), _next_job(state)

	add	$8, %rsp
	xor	returned_job, returned_job
	ret
.endm

ENTRY(aes_cbc_submit_job_inorder_128x8)

	aes_cbc_submit_job_inorder_x8 AES_KEYSIZE_128

ENDPROC(aes_cbc_submit_job_inorder_128x8)

ENTRY(aes_cbc_submit_job_inorder_192x8)

	aes_cbc_submit_job_inorder_x8 AES_KEYSIZE_192

ENDPROC(aes_cbc_submit_job_inorder_192x8)

ENTRY(aes_cbc_submit_job_inorder_256x8)

	aes_cbc_submit_job_inorder_x8 AES_KEYSIZE_256

ENDPROC(aes_cbc_submit_job_inorder_256x8)
