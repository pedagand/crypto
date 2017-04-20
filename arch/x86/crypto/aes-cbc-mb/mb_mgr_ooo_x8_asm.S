/*
 *	AES CBC by8 multibuffer out-of-order scheduler optimization (x86_64)
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

#define arg1	%rdi
#define arg2	%rsi
#define state	arg1
#define job	arg2

/* virtual registers used by aes_cbc_submit_job_ooo_x8 */
#define unused_lanes	%rax
#define lane		%rdx
#define	tmp1		%rcx
#define	tmp2		%r8
#define	tmp3		%r9
#define len		tmp1

#define good_lane	lane

/* virtual registers used by aes_cbc_submit_job_inorder_x8 */
#define new_job		%rdx
#define earliest_job	%rcx
#define minus1		%r8
#define returned_job	%rax	/* register that returns a value from func */

.section .rodata
.align 16
len_masks:
	.octa 0x0000000000000000000000000000FFFF
	.octa 0x000000000000000000000000FFFF0000
	.octa 0x00000000000000000000FFFF00000000
	.octa 0x0000000000000000FFFF000000000000
	.octa 0x000000000000FFFF0000000000000000
	.octa 0x00000000FFFF00000000000000000000
	.octa 0x0000FFFF000000000000000000000000
	.octa 0xFFFF0000000000000000000000000000

dupw:
	.octa 0x01000100010001000100010001000100

one:	.quad  1
two:	.quad  2
three:	.quad  3
four:	.quad  4
five:	.quad  5
six:	.quad  6
seven:	.quad  7

.text

.extern aes_cbc_enc_128_x8
.extern aes_cbc_enc_192_x8
.extern aes_cbc_enc_256_x8

/* arg1/state remains intact after call */
/*
 * void aes_cbc_submit_job__ooo_128x8(
 *	struct aes_cbc_mb_mgr_aes_inorder_x8 *state,
 *	struct job_aes_cbc *job);
 * void aes_cbc_submit_job__ooo_192x8(
 *	struct aes_cbc_mb_mgr_aes_inorder_x8 *state,
 *	struct job_aes_cbc *job);
 * void aes_cbc_submit_job__ooo_256x8(
 *	struct aes_cbc_mb_mgr_aes_inorder_x8 *state,
 *	struct job_aes_cbc *job);
 */

.global aes_cbc_submit_job_ooo_128x8
.global aes_cbc_submit_job_ooo_192x8
.global aes_cbc_submit_job_ooo_256x8


.macro aes_cbc_submit_job_ooo_x8 key_len

	mov	_unused_lanes(state), unused_lanes
	mov	unused_lanes, lane
	and	$0xF, lane
	shr	$4, unused_lanes

	/* state->job_in_lane[lane] = job; */
	mov	job, _job_in_lane(state, lane, 8)

	/*state->lens[lane] = job->len / AES_BLOCK_SIZE; */
	mov	_len(job), DWORD(len)
	shr	$4, len
	mov	WORD(len), _lens(state, lane, 2)

	mov	_plaintext(job), tmp1
	mov	_ciphertext(job), tmp2
	mov	_keys(job), tmp3
	movdqu	_IV(job), %xmm2
	mov	tmp1, _args_in(state, lane, 8)
	mov	tmp2, _args_out(state , lane, 8)
	mov	tmp3, _args_keys(state, lane, 8)
	shl	$4, lane
	movdqa	%xmm2, _args_IV(state , lane)

	movl	$STS_BEING_PROCESSED, _status(job)

	mov	unused_lanes, _unused_lanes(state)
	cmp	$0xF, unused_lanes
	jne	.Lnot_enough_jobs\key_len

	movdqa	_lens(state), %xmm0
	phminposuw	%xmm0, %xmm1
	/*
	 * xmm1{15:0} = min value
	 * xmm1{18:16} = index of min
	 */

	/*
	 * arg1 = rcx = state = args (and it is not clobbered by routine)
	 * arg2 = rdx = min len
	 */
	movd	%xmm1, arg2
	and	$0xFFFF, arg2

	/* subtract min len from lengths */
	pshufb	dupw(%rip), %xmm1	/* duplicate words across all lanes */
	psubw	%xmm1, %xmm0
	movdqa	%xmm0, _lens(state)

	/* need to align stack */
	sub	$8, %rsp
	.if \key_len == AES_KEYSIZE_128
		call aes_cbc_enc_128_x8
	.elseif \key_len == AES_KEYSIZE_192
		call aes_cbc_enc_192_x8
	.elseif \key_len == AES_KEYSIZE_256
		call aes_cbc_enc_256_x8
	.endif
	add	$8, %rsp
	/* arg1/state is still intact */

	/* process completed jobs */
	movdqa	_lens(state), %xmm0
	phminposuw	%xmm0, %xmm1
	/*
	 * xmm1{15:0} = min value
	 * xmm1{18:16} = index of min
	 */

	/*
	 * at this point at least one len should be 0
	 * so min value should be 0
	 * and the index is the index of that lane [0...7]
	 */
	lea	len_masks(%rip), tmp3
	mov	_unused_lanes(state), unused_lanes
	movd	%xmm1, lane
.Lcontinue_loop\key_len:
	/* assert((lane & 0xFFFF) == 0) */
	shr	$16, lane	/* lane is now index */
	mov	_job_in_lane(state, lane, 8), job
	movl	$STS_COMPLETED, _status(job)
	movq	$0, _job_in_lane(state ,lane, 8)
	shl	$4, unused_lanes
	or	lane, unused_lanes
	shl	$4, lane
	movdqa	_args_IV(state,lane), %xmm2
	movdqu	%xmm2, _IV(job)
	por	(tmp3, lane), %xmm0

	phminposuw	%xmm0, %xmm1
	movd	%xmm1, lane
	/* see if bits 15:0 are zero */
	test	$0xFFFF, lane
	jz	.Lcontinue_loop\key_len

	/* done; save registers */
	mov	unused_lanes, _unused_lanes(state)
	/* don't need to save xmm0/lens */

.Lnot_enough_jobs\key_len:
	ret
.endm

ENTRY(aes_cbc_submit_job_ooo_128x8)

	aes_cbc_submit_job_ooo_x8 AES_KEYSIZE_128

ENDPROC(aes_cbc_submit_job_ooo_128x8)

ENTRY(aes_cbc_submit_job_ooo_192x8)

	aes_cbc_submit_job_ooo_x8 AES_KEYSIZE_192

ENDPROC(aes_cbc_submit_job_ooo_192x8)

ENTRY(aes_cbc_submit_job_ooo_256x8)

	aes_cbc_submit_job_ooo_x8 AES_KEYSIZE_256

ENDPROC(aes_cbc_submit_job_ooo_256x8)


/* arg1/state remains intact after call */
/*
 * void aes_cbc_flush_job_ooo_128x8(
 *	struct aes_cbc_mb_mgr_aes_inorder_x8 *state)
 * void aes_cbc_flush_job_ooo_192x8(
 *	struct aes_cbc_mb_mgr_aes_inorder_x8 *state)
 * void aes_cbc_flush_job_ooo_256x8(
 *	struct aes_cbc_mb_mgr_aes_inorder_x8 *state)
 */
.global aes_cbc_flush_job_ooo_128x8
.global aes_cbc_flush_job_ooo_192x8
.global aes_cbc_flush_job_ooo_256x8

.macro aes_cbc_flush_job_ooo_x8 key_len

	mov	_unused_lanes(state), unused_lanes

	/* if bit (32+3) is set, then all lanes are empty */
	bt	$(32+3), unused_lanes
	jc	.Lreturn\key_len

	/* find a lane with a non-null job */
	xor	good_lane, good_lane
	cmpq	$0, (_job_in_lane+8*1)(state)
	cmovne	one(%rip), good_lane
	cmpq	$0, (_job_in_lane+8*2)(state)
	cmovne	two(%rip), good_lane
	cmpq	$0, (_job_in_lane+8*3)(state)
	cmovne	three(%rip), good_lane
	cmpq	$0, (_job_in_lane+8*4)(state)
	cmovne	four(%rip), good_lane
	cmpq	$0, (_job_in_lane+8*5)(state)
	cmovne	five(%rip), good_lane
	cmpq	$0, (_job_in_lane+8*6)(state)
	cmovne	six(%rip), good_lane
	cmpq	$0, (_job_in_lane+8*7)(state)
	cmovne	seven(%rip), good_lane

	/* copy good_lane to empty lanes */
	mov	_args_in(state, good_lane, 8), tmp1
	mov	_args_out(state, good_lane, 8), tmp2
	mov	_args_keys(state, good_lane, 8), tmp3
	shl	$4, good_lane
	movdqa	_args_IV(state, good_lane), %xmm2

	movdqa	_lens(state), %xmm0

	I = 0

.altmacro
	.rept 8
		cmpq	$0, (_job_in_lane + 8*I)(state)
		.if \key_len == AES_KEYSIZE_128
			cond_jump jne, .Lskip128_,%I
		.elseif \key_len == AES_KEYSIZE_192
			cond_jump jne, .Lskip192_,%I
		.elseif \key_len == AES_KEYSIZE_256
			cond_jump jne, .Lskip256_,%I
		.endif
		mov	tmp1, (_args_in + 8*I)(state)
		mov	tmp2, (_args_out + 8*I)(state)
		mov	tmp3, (_args_keys + 8*I)(state)
		movdqa	%xmm2, (_args_IV + 16*I)(state)
		por	(len_masks + 16*I)(%rip), %xmm0
		.if \key_len == AES_KEYSIZE_128
			LABEL .Lskip128_,%I
		.elseif \key_len == AES_KEYSIZE_192
			LABEL .Lskip192_,%I
		.elseif \key_len == AES_KEYSIZE_256
			LABEL .Lskip256_,%I
		.endif
		I = (I+1)
	.endr
.noaltmacro

	phminposuw	%xmm0, %xmm1
	/*
	 * xmm1{15:0} = min value
	 * xmm1{18:16} = index of min
	 */

	/*
	 * arg1 = rcx = state = args (and it is not clobbered by routine)
	 * arg2 = rdx = min len
	 */
	movd	%xmm1, arg2
	and	$0xFFFF, arg2

	/* subtract min len from lengths */
	pshufb	dupw(%rip), %xmm1	/* duplicate words across all lanes */
	psubw	%xmm1, %xmm0
	movdqa	%xmm0, _lens(state)

	/* need to align stack */
	sub	$8, %rsp
	.if \key_len == AES_KEYSIZE_128
		call aes_cbc_enc_128_x8
	.elseif \key_len == AES_KEYSIZE_192
		call aes_cbc_enc_192_x8
	.elseif \key_len == AES_KEYSIZE_256
		call aes_cbc_enc_256_x8
	.endif
	add	$8, %rsp
	/* arg1/state is still intact */

	/* process completed jobs */
	movdqa	_lens(state), %xmm0
	phminposuw	%xmm0, %xmm1
	/*
	 * xmm1{15:0} = min value
	 * xmm1{18:16} = index of min
	 */

	/*
	 * at this point at least one len should be 0, so min value should be 0
	 * and the index is the index of that lane [0...3]
	 */
	lea	len_masks(%rip), tmp3
	mov	_unused_lanes(state), unused_lanes
	movd	%xmm1, lane
.Lcontinue_loop2\key_len:
	/* assert((lane & 0xFFFF) == 0) */
	shr	$16, lane	/* lane is now index */
	mov	_job_in_lane(state, lane, 8), job
	movl	$STS_COMPLETED, _status(job)
	movq	$0, _job_in_lane(state, lane, 8)
	shl	$4, unused_lanes
	or	lane, unused_lanes
	shl	$4, lane
	movdqa	_args_IV(state, lane), %xmm2
	movdqu	%xmm2, _IV(job)
	por	(tmp3, lane), %xmm0

	phminposuw	%xmm0, %xmm1
	movd	%xmm1, lane
	/* see if bits 15:0 are zero */
	test	$0xFFFF, lane
	jz	.Lcontinue_loop2\key_len

	/* done; save registers */
	mov	unused_lanes, _unused_lanes(state)
	/* don't need to save xmm0/lens */
.Lreturn\key_len:
	ret
.endm

ENTRY(aes_cbc_flush_job_ooo_128x8)

	aes_cbc_flush_job_ooo_x8 AES_KEYSIZE_128

ENDPROC(aes_cbc_flush_job_ooo_128x8)

ENTRY(aes_cbc_flush_job_ooo_192x8)

	aes_cbc_flush_job_ooo_x8 AES_KEYSIZE_192

ENDPROC(aes_cbc_flush_job_ooo_192x8)

ENTRY(aes_cbc_flush_job_ooo_256x8)

	aes_cbc_flush_job_ooo_x8 AES_KEYSIZE_256

ENDPROC(aes_cbc_flush_job_ooo_256x8)
