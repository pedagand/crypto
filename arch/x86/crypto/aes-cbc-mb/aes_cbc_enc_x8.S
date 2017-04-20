/*
 *	AES CBC by8 multibuffer optimization (x86_64)
 *	This file implements 128/192/256 bit AES CBC encryption
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

/* stack size needs to be an odd multiple of 8 for alignment */

#define AES_KEYSIZE_128        16
#define AES_KEYSIZE_192        24
#define AES_KEYSIZE_256        32

#define XMM_SAVE_SIZE	16*10
#define GPR_SAVE_SIZE	8*9
#define STACK_SIZE	(XMM_SAVE_SIZE + GPR_SAVE_SIZE)

#define GPR_SAVE_REG	%rsp
#define GPR_SAVE_AREA	%rsp + XMM_SAVE_SIZE
#define LEN_AREA_OFFSET	XMM_SAVE_SIZE + 8*8
#define LEN_AREA_REG	%rsp
#define LEN_AREA	%rsp + XMM_SAVE_SIZE + 8*8

#define IN_OFFSET	0
#define OUT_OFFSET	8*8
#define KEYS_OFFSET	16*8
#define IV_OFFSET	24*8


#define IDX	%rax
#define TMP	%rbx
#define ARG	%rdi
#define LEN	%rsi

#define KEYS0	%r14
#define KEYS1	%r15
#define KEYS2	%rbp
#define KEYS3	%rdx
#define KEYS4	%rcx
#define KEYS5	%r8
#define KEYS6	%r9
#define KEYS7	%r10

#define IN0	%r11
#define IN2	%r12
#define IN4	%r13
#define IN6	LEN

#define XDATA0	%xmm0
#define XDATA1	%xmm1
#define XDATA2	%xmm2
#define XDATA3	%xmm3
#define XDATA4	%xmm4
#define XDATA5	%xmm5
#define XDATA6	%xmm6
#define XDATA7	%xmm7

#define XKEY0_3	%xmm8
#define XKEY1_4	%xmm9
#define XKEY2_5	%xmm10
#define XKEY3_6	%xmm11
#define XKEY4_7	%xmm12
#define XKEY5_8	%xmm13
#define XKEY6_9	%xmm14
#define XTMP	%xmm15

#define	MOVDQ movdqu /* assume buffers not aligned */
#define CONCAT(a, b)	a##b
#define INPUT_REG_SUFX	1	/* IN */
#define XDATA_REG_SUFX	2	/* XDAT */
#define KEY_REG_SUFX	3	/* KEY */
#define XMM_REG_SUFX	4	/* XMM */

/*
 * To avoid positional parameter errors while compiling
 * three registers need to be passed
 */
.text

.macro pxor2 x, y, z
	MOVDQ	(\x,\y), XTMP
	pxor	XTMP, \z
.endm

.macro inreg n
	.if (\n == 0)
		reg_IN = IN0
	.elseif (\n == 2)
		reg_IN = IN2
	.elseif (\n == 4)
		reg_IN = IN4
	.elseif (\n == 6)
		reg_IN = IN6
	.else
		error "inreg: incorrect register number"
	.endif
.endm
.macro xdatareg n
	.if (\n == 0)
		reg_XDAT = XDATA0
	.elseif (\n == 1)
		reg_XDAT = XDATA1
	.elseif (\n == 2)
		reg_XDAT = XDATA2
	.elseif (\n == 3)
		reg_XDAT = XDATA3
	.elseif (\n == 4)
		reg_XDAT = XDATA4
	.elseif (\n == 5)
		reg_XDAT = XDATA5
	.elseif (\n == 6)
		reg_XDAT = XDATA6
	.elseif (\n == 7)
		reg_XDAT = XDATA7
	.endif
.endm
.macro xkeyreg n
	.if (\n == 0)
		reg_KEY = KEYS0
	.elseif (\n == 1)
		reg_KEY = KEYS1
	.elseif (\n == 2)
		reg_KEY = KEYS2
	.elseif (\n == 3)
		reg_KEY = KEYS3
	.elseif (\n == 4)
		reg_KEY = KEYS4
	.elseif (\n == 5)
		reg_KEY = KEYS5
	.elseif (\n == 6)
		reg_KEY = KEYS6
	.elseif (\n == 7)
		reg_KEY = KEYS7
	.endif
.endm
.macro xmmreg n
	.if (\n >= 0) && (\n < 16)
		/* Valid register number */
		reg_XMM = %xmm\n
	.else
		error "xmmreg: incorrect register number"
	.endif
.endm

/*
 * suffix - register suffix
 * set up the register name using the loop index I
 */
.macro define_reg suffix
.altmacro
	.if (\suffix == INPUT_REG_SUFX)
		inreg  %I
	.elseif (\suffix == XDATA_REG_SUFX)
		xdatareg  %I
	.elseif (\suffix == KEY_REG_SUFX)
		xkeyreg  %I
	.elseif (\suffix == XMM_REG_SUFX)
		xmmreg  %I
	.else
		error "define_reg: unknown register suffix"
	.endif
.noaltmacro
.endm

/*
 * aes_cbc_enc_x8 key_len
 * macro to encode data for 128bit, 192bit and 256bit keys
 */

.macro aes_cbc_enc_x8 key_len

	sub	$STACK_SIZE, %rsp

	mov	%rbx, (XMM_SAVE_SIZE + 8*0)(GPR_SAVE_REG)
	mov	%rbp, (XMM_SAVE_SIZE + 8*3)(GPR_SAVE_REG)
	mov	%r12, (XMM_SAVE_SIZE + 8*4)(GPR_SAVE_REG)
	mov	%r13, (XMM_SAVE_SIZE + 8*5)(GPR_SAVE_REG)
	mov	%r14, (XMM_SAVE_SIZE + 8*6)(GPR_SAVE_REG)
	mov	%r15, (XMM_SAVE_SIZE + 8*7)(GPR_SAVE_REG)

	mov	$16, IDX
	shl	$4, LEN	/* LEN = LEN * 16 */
	/* LEN is now in terms of bytes */
	mov	LEN, (LEN_AREA_OFFSET)(LEN_AREA_REG)

	/* Run through storing arguments in IN0,2,4,6 */
	I = 0
	.rept 4
		define_reg	INPUT_REG_SUFX
		mov	(IN_OFFSET + 8*I)(ARG), reg_IN
		I = (I + 2)
	.endr

	/* load 1 .. 8 blocks of plain text into XDATA0..XDATA7 */
	I = 0
	.rept 4
		mov		(IN_OFFSET + 8*(I+1))(ARG), TMP
		define_reg	INPUT_REG_SUFX
		define_reg	XDATA_REG_SUFX
		/* load first block of plain text */
		MOVDQ		(reg_IN), reg_XDAT
		I = (I + 1)
		define_reg	XDATA_REG_SUFX
		/* load next block of plain text */
		MOVDQ		(TMP), reg_XDAT
		I = (I + 1)
	.endr

	/* Run through XDATA0 .. XDATA7 to perform plaintext XOR IV */
	I = 0
	.rept 8
		define_reg	XDATA_REG_SUFX
		pxor	(IV_OFFSET + 16*I)(ARG), reg_XDAT
		I = (I + 1)
	.endr

	I = 0
	.rept 8
		define_reg	KEY_REG_SUFX
		mov	(KEYS_OFFSET + 8*I)(ARG), reg_KEY
		I = (I + 1)
	.endr

	I = 0
	/* 0..7 ARK */
	.rept 8
		define_reg	XDATA_REG_SUFX
		define_reg	KEY_REG_SUFX
		pxor	16*0(reg_KEY), reg_XDAT
		I = (I + 1)
	.endr

	I = 0
		/* 1. ENC */
	.rept 8
		define_reg	XDATA_REG_SUFX
		define_reg	KEY_REG_SUFX
		aesenc	(16*1)(reg_KEY), reg_XDAT
		I = (I + 1)
	.endr

	movdqa		16*3(KEYS0), XKEY0_3	/* load round 3 key */

	I = 0
		/* 2. ENC */
	.rept 8
		define_reg	XDATA_REG_SUFX
		define_reg	KEY_REG_SUFX
		aesenc	(16*2)(reg_KEY), reg_XDAT
		I = (I + 1)
	.endr

	movdqa		16*4(KEYS1), XKEY1_4	/* load round 4 key */

		/* 3. ENC */
	aesenc		XKEY0_3, XDATA0
	I = 1
	.rept 7
		define_reg	XDATA_REG_SUFX
		define_reg	KEY_REG_SUFX
		aesenc	(16*3)(reg_KEY), reg_XDAT
		I = (I + 1)
	.endr

	/*
	 * FIXME:
	 * why can't we reorder encrypt DATA0..DATA7 and load 5th round?
	 */
	aesenc		(16*4)(KEYS0), XDATA0		/* 4. ENC */
	movdqa		16*5(KEYS2), XKEY2_5		/* load round 5 key */
	aesenc		XKEY1_4, XDATA1			/* 4. ENC */

	I = 2
	.rept 6
		define_reg	XDATA_REG_SUFX
		define_reg	KEY_REG_SUFX
		aesenc	(16*4)(reg_KEY), reg_XDAT
		I = (I + 1)
	.endr

	aesenc		(16*5)(KEYS0), XDATA0		/* 5. ENC */
	aesenc		(16*5)(KEYS1), XDATA1		/* 5. ENC */
	movdqa		16*6(KEYS3), XKEY3_6		/* load round 6 key */
	aesenc		XKEY2_5, XDATA2			/* 5. ENC */
	I = 3
	.rept 5
		define_reg	XDATA_REG_SUFX
		define_reg	KEY_REG_SUFX
		aesenc	(16*5)(reg_KEY), reg_XDAT
		I = (I + 1)
	.endr

	aesenc		(16*6)(KEYS0), XDATA0		/* 6. ENC */
	aesenc		(16*6)(KEYS1), XDATA1		/* 6. ENC */
	aesenc		(16*6)(KEYS2), XDATA2		/* 6. ENC */
	movdqa		16*7(KEYS4), XKEY4_7		/* load round 7 key */
	aesenc		XKEY3_6, XDATA3			/* 6. ENC */

	I = 4
	.rept 4
		define_reg	XDATA_REG_SUFX
		define_reg	KEY_REG_SUFX
		aesenc	(16*6)(reg_KEY), reg_XDAT
		I = (I + 1)
	.endr

	I = 0
	.rept 4
		define_reg	XDATA_REG_SUFX
		define_reg	KEY_REG_SUFX
		aesenc	(16*7)(reg_KEY), reg_XDAT
		I = (I + 1)
	.endr
	movdqa		16*8(KEYS5), XKEY5_8		/* load round 8 key */
	aesenc		XKEY4_7, XDATA4			/* 7. ENC */
	I = 5
	.rept 3
		define_reg	XDATA_REG_SUFX
		define_reg	KEY_REG_SUFX
		aesenc	(16*7)(reg_KEY), reg_XDAT
		I = (I + 1)
	.endr

	I = 0
	.rept 5
		define_reg	XDATA_REG_SUFX
		define_reg	KEY_REG_SUFX
		aesenc	(16*8)(reg_KEY), reg_XDAT
		I = (I + 1)
	.endr
	movdqa		16*9(KEYS6), XKEY6_9		/* load round 9 key */
	aesenc		XKEY5_8, XDATA5			/* 8. ENC */
	aesenc		16*8(KEYS6), XDATA6		/* 8. ENC */
	aesenc		16*8(KEYS7), XDATA7		/* 8. ENC */

	I = 0
	.rept 6
		define_reg	XDATA_REG_SUFX
		define_reg	KEY_REG_SUFX
		aesenc	(16*9)(reg_KEY), reg_XDAT
		I = (I + 1)
	.endr
	mov		(OUT_OFFSET + 8*0)(ARG), TMP
	aesenc		XKEY6_9, XDATA6			/* 9. ENC */
	aesenc		16*9(KEYS7), XDATA7		/* 9. ENC */

		/* 10. ENC (last for 128bit keys) */
	I = 0
	.rept 8
		define_reg	XDATA_REG_SUFX
		define_reg	KEY_REG_SUFX
		.if (\key_len == AES_KEYSIZE_128)
			aesenclast	(16*10)(reg_KEY), reg_XDAT
		.else
			aesenc	(16*10)(reg_KEY), reg_XDAT
		.endif
		I = (I + 1)
	.endr

	.if (\key_len != AES_KEYSIZE_128)
		/* 11. ENC */
		I = 0
		.rept 8
			define_reg	XDATA_REG_SUFX
			define_reg	KEY_REG_SUFX
			aesenc	(16*11)(reg_KEY), reg_XDAT
			I = (I + 1)
		.endr

		/* 12. ENC (last for 192bit key) */
		I = 0
		.rept 8
			define_reg	XDATA_REG_SUFX
			define_reg	KEY_REG_SUFX
			.if (\key_len == AES_KEYSIZE_192)
				aesenclast	(16*12)(reg_KEY), reg_XDAT
			.else
				aesenc		(16*12)(reg_KEY), reg_XDAT
			.endif
			I = (I + 1)
		.endr

		/* for 256bit, two more rounds */
		.if \key_len == AES_KEYSIZE_256

			/* 13. ENC */
			I = 0
			.rept 8
				define_reg	XDATA_REG_SUFX
				define_reg	KEY_REG_SUFX
				aesenc	(16*13)(reg_KEY), reg_XDAT
				I = (I + 1)
			.endr

			/* 14. ENC last encode for 256bit key */
			I = 0
			.rept 8
				define_reg	XDATA_REG_SUFX
				define_reg	KEY_REG_SUFX
				aesenclast	(16*14)(reg_KEY), reg_XDAT
				I = (I + 1)
			.endr
		.endif

	.endif

	I = 0
	.rept 8
		define_reg	XDATA_REG_SUFX
		MOVDQ	reg_XDAT, (TMP)	/* write back ciphertext */
		I = (I + 1)
		.if (I < 8)
			mov	(OUT_OFFSET + 8*I)(ARG), TMP
		.endif
	.endr

	cmp		IDX, LEN_AREA_OFFSET(LEN_AREA_REG)
	je		.Ldone\key_len

.Lmain_loop\key_len:
	mov		(IN_OFFSET + 8*1)(ARG), TMP
	pxor2		IN0, IDX, XDATA0	/* next block of plain text */
	pxor2		TMP, IDX, XDATA1	/* next block of plain text */

	mov		(IN_OFFSET + 8*3)(ARG), TMP
	pxor2		IN2, IDX, XDATA2	/* next block of plain text */
	pxor2		TMP, IDX, XDATA3	/* next block of plain text */

	mov		(IN_OFFSET + 8*5)(ARG), TMP
	pxor2		IN4, IDX, XDATA4	/* next block of plain text */
	pxor2		TMP, IDX, XDATA5	/* next block of plain text */

	mov		(IN_OFFSET + 8*7)(ARG), TMP
	pxor2		IN6, IDX, XDATA6	/* next block of plain text */
	pxor2		TMP, IDX, XDATA7	/* next block of plain text */

	/* 0. ARK */
	I = 0
	.rept 8
		define_reg	XDATA_REG_SUFX
		define_reg	KEY_REG_SUFX
		pxor	16*0(reg_KEY), reg_XDAT
		I = (I + 1)
	.endr

	/* 1. ENC */
	I = 0
	.rept 8
		define_reg	XDATA_REG_SUFX
		define_reg	KEY_REG_SUFX
		aesenc	16*1(reg_KEY), reg_XDAT
		I = (I + 1)
	.endr

	/* 2. ENC */
	I = 0
	.rept 8
		define_reg	XDATA_REG_SUFX
		define_reg	KEY_REG_SUFX
		aesenc	16*2(reg_KEY), reg_XDAT
		I = (I + 1)
	.endr

	/* 3. ENC */
	aesenc		XKEY0_3, XDATA0		/* 3. ENC */
	I = 1
	.rept 7
		define_reg	XDATA_REG_SUFX
		define_reg	KEY_REG_SUFX
		aesenc	16*3(reg_KEY), reg_XDAT
		I = (I + 1)
	.endr

	/* 4. ENC */
	aesenc		16*4(KEYS0), XDATA0	/* 4. ENC */
	aesenc		XKEY1_4, XDATA1		/* 4. ENC */
	I = 2
	.rept 6
		define_reg	XDATA_REG_SUFX
		define_reg	KEY_REG_SUFX
		aesenc	16*4(reg_KEY), reg_XDAT
		I = (I + 1)
	.endr

	/* 5. ENC */
	aesenc		16*5(KEYS0), XDATA0	/* 5. ENC */
	aesenc		16*5(KEYS1), XDATA1	/* 5. ENC */
	aesenc		XKEY2_5, XDATA2		/* 5. ENC */

	I = 3
	.rept 5
		define_reg	XDATA_REG_SUFX
		define_reg	KEY_REG_SUFX
		aesenc	16*5(reg_KEY), reg_XDAT
		I = (I + 1)
	.endr

	/* 6. ENC */
	I = 0
	.rept 3
		define_reg	XDATA_REG_SUFX
		define_reg	KEY_REG_SUFX
		aesenc	16*6(reg_KEY), reg_XDAT
		I = (I + 1)
	.endr
	aesenc		XKEY3_6, XDATA3		/* 6. ENC */
	I = 4
	.rept 4
		define_reg	XDATA_REG_SUFX
		define_reg	KEY_REG_SUFX
		aesenc	16*6(reg_KEY), reg_XDAT
		I = (I + 1)
	.endr

	/* 7. ENC */
	I = 0
	.rept 4
		define_reg	XDATA_REG_SUFX
		define_reg	KEY_REG_SUFX
		aesenc	16*7(reg_KEY), reg_XDAT
		I = (I + 1)
	.endr
	aesenc		XKEY4_7, XDATA4		/* 7. ENC */
	I = 5
	.rept 3
		define_reg	XDATA_REG_SUFX
		define_reg	KEY_REG_SUFX
		aesenc	16*7(reg_KEY), reg_XDAT
		I = (I + 1)
	.endr

	/* 8. ENC */
	I = 0
	.rept 5
		define_reg	XDATA_REG_SUFX
		define_reg	KEY_REG_SUFX
		aesenc	16*8(reg_KEY), reg_XDAT
		I = (I + 1)
	.endr
	aesenc		XKEY5_8, XDATA5		/* 8. ENC */
	aesenc		16*8(KEYS6), XDATA6	/* 8. ENC */
	aesenc		16*8(KEYS7), XDATA7	/* 8. ENC */

	/* 9. ENC */
	I = 0
	.rept 6
		define_reg	XDATA_REG_SUFX
		define_reg	KEY_REG_SUFX
		aesenc	16*9(reg_KEY), reg_XDAT
		I = (I + 1)
	.endr
	mov		(OUT_OFFSET + 8*0)(ARG), TMP
	aesenc		XKEY6_9, XDATA6		/* 9. ENC */
	aesenc		16*9(KEYS7), XDATA7	/* 9. ENC */

	/* 10. ENC (last for 128 bit key) */
	I = 0
	.rept 8
		define_reg	XDATA_REG_SUFX
		define_reg	KEY_REG_SUFX
		.if (\key_len == AES_KEYSIZE_128)
			aesenclast	16*10(reg_KEY), reg_XDAT
		.else
			aesenc	16*10(reg_KEY), reg_XDAT
		.endif
		I = (I + 1)
	.endr

	.if (\key_len != AES_KEYSIZE_128)
		/* 11. ENC */
		I = 0
		.rept 8
			define_reg	XDATA_REG_SUFX
			define_reg	KEY_REG_SUFX
			aesenc	16*11(reg_KEY), reg_XDAT
			I = (I + 1)
		.endr

		/* 12. last ENC for 192bit key */
		I = 0
		.rept 8
			define_reg	XDATA_REG_SUFX
			define_reg	KEY_REG_SUFX
			.if (\key_len == AES_KEYSIZE_192)
				aesenclast	16*12(reg_KEY), reg_XDAT
			.else
				aesenc		16*12(reg_KEY), reg_XDAT
			.endif
			I = (I + 1)
		.endr

		.if \key_len == AES_KEYSIZE_256
			/* for 256bit, two more rounds */
			/* 13. ENC */
			I = 0
			.rept 8
				define_reg	XDATA_REG_SUFX
				define_reg	KEY_REG_SUFX
				aesenc	16*13(reg_KEY), reg_XDAT
				I = (I + 1)
			.endr

			/* 14. last ENC for 256bit key */
			I = 0
			.rept 8
				define_reg	XDATA_REG_SUFX
				define_reg	KEY_REG_SUFX
				aesenclast	16*14(reg_KEY), reg_XDAT
				I = (I + 1)
			.endr
		.endif
	.endif

	I = 0
	.rept 8
		define_reg	XDATA_REG_SUFX
		/* write back cipher text */
		MOVDQ		reg_XDAT, (TMP , IDX)
		I = (I + 1)
		.if (I < 8)
			mov		(OUT_OFFSET + 8*I)(ARG), TMP
		.endif
	.endr

	add	$16, IDX
	cmp	IDX, LEN_AREA_OFFSET(LEN_AREA_REG)
	jne	.Lmain_loop\key_len

.Ldone\key_len:
	/* update IV */
	I = 0
	.rept 8
		define_reg	XDATA_REG_SUFX
		movdqa	reg_XDAT, (IV_OFFSET + 16*I)(ARG)
		I = (I + 1)
	.endr

	/* update IN and OUT */
	movd	LEN_AREA_OFFSET(LEN_AREA_REG), %xmm0
	pshufd	$0x44, %xmm0, %xmm0

	I = 1
	.rept 4
		define_reg	XMM_REG_SUFX
		movdqa	(IN_OFFSET + 16*(I-1))(ARG), reg_XMM
		I = (I + 1)
	.endr

	paddq	%xmm0, %xmm1
	paddq	%xmm0, %xmm2
	paddq	%xmm0, %xmm3
	paddq	%xmm0, %xmm4

	I = 5
	.rept 4
		define_reg	XMM_REG_SUFX
		movdqa	(OUT_OFFSET + 16*(I-5))(ARG), reg_XMM
		I = (I + 1)
	.endr

	I = 1
	.rept 4
		define_reg	XMM_REG_SUFX
		movdqa	reg_XMM, (IN_OFFSET + 16*(I-1))(ARG)
		I = (I + 1)
	.endr

	paddq	%xmm0, %xmm5
	paddq	%xmm0, %xmm6
	paddq	%xmm0, %xmm7
	paddq	%xmm0, %xmm8

	I = 5
	.rept 4
		define_reg	XMM_REG_SUFX
		movdqa	reg_XMM, (OUT_OFFSET + 16*(I-5))(ARG)
		I = (I + 1)
	.endr

	mov	(XMM_SAVE_SIZE + 8*0)(GPR_SAVE_REG), %rbx
	mov	(XMM_SAVE_SIZE + 8*3)(GPR_SAVE_REG), %rbp
	mov	(XMM_SAVE_SIZE + 8*4)(GPR_SAVE_REG), %r12
	mov	(XMM_SAVE_SIZE + 8*5)(GPR_SAVE_REG), %r13
	mov	(XMM_SAVE_SIZE + 8*6)(GPR_SAVE_REG), %r14
	mov	(XMM_SAVE_SIZE + 8*7)(GPR_SAVE_REG), %r15

	add	$STACK_SIZE, %rsp

	ret
.endm

/*
 * AES CBC encryption routine supporting 128/192/256 bit keys
 *
 * void aes_cbc_enc_128_x8(struct aes_cbc_args_x8 *args, u64 len);
 * arg 1: rcx : addr of AES_ARGS_x8 structure
 * arg 2: rdx : len (in units of 16-byte blocks)
 * void aes_cbc_enc_192_x8(struct aes_cbc_args_x8 *args, u64 len);
 * arg 1: rcx : addr of aes_cbc_args_x8 structure
 * arg 2: rdx : len (in units of 16-byte blocks)
 * void aes_cbc_enc_256_x8(struct aes_cbc_args_x8 *args, u64 len);
 * arg 1: rcx : addr of aes_cbc_args_x8 structure
 * arg 2: rdx : len (in units of 16-byte blocks)
 */

ENTRY(aes_cbc_enc_128_x8)

	aes_cbc_enc_x8 AES_KEYSIZE_128

ENDPROC(aes_cbc_enc_128_x8)

ENTRY(aes_cbc_enc_192_x8)

	aes_cbc_enc_x8 AES_KEYSIZE_192

ENDPROC(aes_cbc_enc_192_x8)

ENTRY(aes_cbc_enc_256_x8)

	aes_cbc_enc_x8 AES_KEYSIZE_256

ENDPROC(aes_cbc_enc_256_x8)
