/**
 * This file contains basic ioctls to user space and data structures used.
 *
 * Copyright (C) 2011, Texas Instruments, Incorporated
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */


#ifndef __EP_EDMA__
#define __EP_EDMA__

#undef u8
#undef u32

#define u8 unsigned char
#define u32 unsigned int

/*
 * TODO: Use correct types for ioctls
 */

/* ioctls defined for driver as well as user space */

#define TI81XX_EDMA_WRITE                      _IOWR('P', 1, unsigned int)
#define TI81XX_EDMA_READ                       _IOWR('P', 2, unsigned int)
#define TI81XX_EDMA_WRITEM		       _IOWR('P', 3, unsigned int)
#define TI81XX_EDMA_READM		       _IOWR('P', 4, unsigned int)
#define TI81XX_EDMA_SET_CNT			_IOWR('P', 5, unsigned int)
#define TI81XX_EDMA_SET_BUF_INFO		_IOWR('P', 6, unsigned int)
#define TI81XX_EDMA_WRITE_DIRECT		       _IOWR('P', 7, unsigned int)
#define TI81XX_EDMA_READ_DIRECT		       _IOWR('P', 8, unsigned int)
#define KS2_EDMA_WRITE_DIRECT		       _IOWR('P', 9, unsigned int)
#define KS2_EDMA_READ_DIRECT		       _IOWR('P', 10, unsigned int)



/**
 * dma_info -  EDMA related configuration data
 * @size: size of data to be transfered
 * @user_buf: pointer to user space buffer
 * @dset: destination address for DMA
 * @src: source address for DMA
 * @dir: direction of DMA ( IF DMA to some peer, src
 *					will be NULL if DMA from
 *						some peer, dest will be NULL)
 */

struct dma_info {
	u32 size;
	u8  *user_buf;
	u32 dest;
	u32 src;
	u32 dir;
};

/**
 * dma_cnt_inf- configuration of edma.
 */

struct dma_cnt_conf {
	int acnt;
	int bcnt;
	int ccnt;
	int mode;
	int cord_idx;
};

typedef struct _ks2_edma_info {
	struct dma_info edma;
	struct dma_cnt_conf cnt;
}ks2_edma_info;


/**
 * dma_buf_info- buffer related info
 */

struct dma_buf_info {
	u32 send_buf;
	u32 recv_buf;
	u32 size;
};

/**
 * start and end address of memory used for edma buffers
 */

extern u32 ti81xx_edma_mem_start;
extern u32 ti81xx_edma_mem_size;

#endif
