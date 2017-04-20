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
#ifndef __AES_CBC_MB_CTX_H
#define __AES_CBC_MB_CTX_H


#include <linux/types.h>

#include "aes_cbc_mb_mgr.h"

#define CBC_ENCRYPT	0x01
#define CBC_DECRYPT	0x02
#define CBC_START	0x04
#define CBC_DONE	0x08

#define CBC_CTX_STS_IDLE       0x00
#define CBC_CTX_STS_PROCESSING 0x01
#define CBC_CTX_STS_LAST       0x02
#define CBC_CTX_STS_COMPLETE   0x04

enum cbc_ctx_error {
	CBC_CTX_ERROR_NONE               =  0,
	CBC_CTX_ERROR_INVALID_FLAGS      = -1,
	CBC_CTX_ERROR_ALREADY_PROCESSING = -2,
	CBC_CTX_ERROR_ALREADY_COMPLETED  = -3,
};

#define cbc_ctx_init(ctx, n_bytes, op) \
	do { \
		(ctx)->flag = (op) | CBC_START; \
		(ctx)->nbytes = (n_bytes); \
	} while (0)

/* AESNI routines to perform cbc decrypt and key expansion */

asmlinkage void aesni_cbc_dec(struct crypto_aes_ctx *ctx, u8 *out,
			      const u8 *in, unsigned int len, u8 *iv);
asmlinkage int aesni_set_key(struct crypto_aes_ctx *ctx, const u8 *in_key,
			     unsigned int key_len);

#endif /* __AES_CBC_MB_CTX_H */
