#
# Arch-specific CryptoAPI modules.
#

avx_supported := $(call as-instr,vpxor %xmm0$(comma)%xmm0$(comma)%xmm0,yes,no)

# we need decryption and key expansion routine symbols
# if either AESNI_NI_INTEL or AES_CBC_MB is a module

ifeq ($(CONFIG_CRYPTO_AES_NI_INTEL),m)
	dec_support := ../aesni-intel_asm.o
endif
ifeq ($(CONFIG_CRYPTO_AES_CBC_MB),m)
	dec_support := ../aesni-intel_asm.o
endif

ifeq ($(avx_supported),yes)
	obj-$(CONFIG_CRYPTO_AES_CBC_MB) += aes-cbc-mb.o
	aes-cbc-mb-y := $(dec_support) aes_cbc_mb.o aes_mb_mgr_init.o \
			mb_mgr_inorder_x8_asm.o mb_mgr_ooo_x8_asm.o \
			aes_cbc_enc_x8.o
endif
