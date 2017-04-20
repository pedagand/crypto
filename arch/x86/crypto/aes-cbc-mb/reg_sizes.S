/*
 *	Header file AES CBC multibuffer SSE optimization (x86_64)
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
/* define d and w variants for registers */

#define	raxd	eax
#define raxw	ax
#define raxb	al

#define	rbxd	ebx
#define rbxw	bx
#define rbxb	bl

#define	rcxd	ecx
#define rcxw	cx
#define rcxb	cl

#define	rdxd	edx
#define rdxw	dx
#define rdxb	dl

#define	rsid	esi
#define rsiw	si
#define rsib	sil

#define	rdid	edi
#define rdiw	di
#define rdib	dil

#define	rbpd	ebp
#define rbpw	bp
#define rbpb	bpl

#define ymm0x	%xmm0
#define ymm1x	%xmm1
#define ymm2x	%xmm2
#define ymm3x	%xmm3
#define ymm4x	%xmm4
#define ymm5x	%xmm5
#define ymm6x	%xmm6
#define ymm7x	%xmm7
#define ymm8x	%xmm8
#define ymm9x	%xmm9
#define ymm10x	%xmm10
#define ymm11x	%xmm11
#define ymm12x	%xmm12
#define ymm13x	%xmm13
#define ymm14x	%xmm14
#define ymm15x	%xmm15

#define CONCAT(a,b)	a##b
#define DWORD(reg)	CONCAT(reg, d)
#define WORD(reg)	CONCAT(reg, w)
#define BYTE(reg)	CONCAT(reg, b)

#define XWORD(reg)	CONCAT(reg,x)

/* common macros */

/* Generate a label to go to */
.macro LABEL prefix, num
\prefix\num\():
.endm

/*
 * cond_jump ins, name, suffix
 * ins - conditional jump instruction to execute
 * name,suffix - concatenate to form the label to go to
 */
.macro cond_jump ins, name, suffix
	\ins	\name\suffix
.endm
