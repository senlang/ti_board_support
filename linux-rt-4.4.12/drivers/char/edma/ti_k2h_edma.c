/*
 * TI816X/TI814X PCIe Endpoint (EP) edma kernel module to provide EDMA interface
 * to user space.
 *
 * Note: References common across TI816X and TI814X devices are fererred as
 * TI81XX/ti81xx.
 *
 * Copyright (C) 2010 Texas Instruments, Incorporated
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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
//#include <mach/irqs.h>
//#include <mach/hardware.h>
//#include <memory.h>
#include <linux/cdev.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/fcntl.h>
#include <linux/uaccess.h>
#include <linux/ioport.h>
#include <linux/ioport.h>
#include <linux/jiffies.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/platform_data/ti_edma.h>
#include <linux/interrupt.h>
#include <linux/completion.h>
#include <linux/delay.h>


#include "ti_k2h_edma.h"


#define MAX_DMA_TRANSFER_IN_BYTES   (32768)
#define STATIC_SHIFT                3
#define TCINTEN_SHIFT               20
#define ITCINTEN_SHIFT              21
#define TCCHEN_SHIFT                22
#define ITCCHEN_SHIFT               23


#define DEV_MINOR			0
#define DEVICENO			1
#define DRIVER_NAME			"ti_k2h_edma"
#define TI81XX_DMA_MODFILE		"ti_k2h_edma"

/*direction of DMA */

#define EP_OUT				0
#define EP_IN				1

#define MAX_CORE_NUM		4

dev_t ti81xx_ep_dma_dev;

/****************************************************************************
 *	FILE GLOBALS
 */


static int irqraised1[MAX_CORE_NUM];
static int irqraised2[MAX_CORE_NUM];

static int acnt = (32 * 1024);
static int bcnt = 1;
static int ccnt = 1;
static int mode = ABSYNC;
static struct cdev ti81xx_ep_dma_cdev;
static struct class *ti81xx_dma_class;

/*source buffer on local EP for DMA write to some Remote peer*/
dma_addr_t dmaphyssrcep;
dma_addr_t dmaphyssrcrm;
/*dest buffer on local EP for DMA read from Remote peer*/
dma_addr_t dmaphysdestep;
dma_addr_t dmaphysdestrm;

char *dmabufsrcep;
char *dmabufsrcrm;
char *dmabufdestep;
char *dmabufdestrm;

wait_queue_head_t read_queue0;
wait_queue_head_t read_queue1;
wait_queue_head_t read_queue2;
wait_queue_head_t read_queue3;

wait_queue_head_t write_queue[MAX_CORE_NUM];
wait_queue_head_t read_queue[MAX_CORE_NUM];


struct semaphore read_sema0;
struct semaphore read_sema1;

struct completion read_completion0;
struct completion read_completion1;
struct completion read_completion2;
struct completion read_completion3;

/*****************************************************************************
 *	FUNCTION DECLARATIONS
 */

static int ti81xx_edma_dma_tx(int acnt, int bcnt, int ccnt,
					int sync_mode, int event_queue);

static int ti81xx_edma_dma_rx(int acnt, int bcnt, int ccnt,
					int sync_mode, int event_queue);

static void callback1(unsigned channel, u16 ch_status, void *data);
static void callback2(unsigned channel, u16 ch_status, void *data);
static long ti81xx_ep_dma_ioctl(struct file *file,
					unsigned int cmd, unsigned long arg);

static int ti81xx_edma_dma_tx_direct(int acnt, int bcnt, int ccnt, int sync_mode,
							int event_queue,dma_addr_t src,dma_addr_t dest);
static int ti81xx_edma_dma_rx_direct(int acnt, int bcnt, int ccnt, int core_idx,
							int event_queue,dma_addr_t src,dma_addr_t dest);

/*****************************************************************************
 *	FUNCTION DEFINITIONS
 */



static void callback3(unsigned ctlr, unsigned channel, u16 ch_status, void *data)
{
	//printk("%s[%d]%d,0x%x\n",__FUNCTION__,__LINE__,ctlr,channel);
	
	switch (ch_status) {
	case DMA_COMPLETE:
		irqraised2[ctlr] = 1;

		wake_up_interruptible(&read_queue[ctlr]);

		pr_debug(DRIVER_NAME
				":  From Callback : Channel %d status is: %u",
				channel, ch_status);

		break;
		
	case DMA_CC_ERROR:
		irqraised2[ctlr] = -1;

		pr_debug(DRIVER_NAME ":  From Callback : DMA_EVT_MISS_ERROR "
				"occured on Channel %d", channel);

		break;
	default:
		break;
	}
}

static void callback4(unsigned ctlr, unsigned channel, u16 ch_status, void *data)
{
	//printk("%s[%d]\n",__FUNCTION__,__LINE__);
	
	switch (ch_status) {
	case DMA_COMPLETE:
		irqraised1[ctlr] = 1;
		//printk("%s[%d]\n",__FUNCTION__,__LINE__);
		wake_up_interruptible(&write_queue[ctlr]);
		pr_debug(DRIVER_NAME
				":  From Callback : Channel %d status is: %u",
				channel, ch_status);
		break;
	case DMA_CC_ERROR:
		irqraised1[ctlr] = -1;
		//printk("%s[%d]\n",__FUNCTION__,__LINE__);
		pr_debug(DRIVER_NAME ":  From Callback : DMA_EVT_MISS_ERROR "
				" occured on Channel %d", channel);
		break;
	default:
		break;
	}
}




/**
 * ti81xx_ep_dma_ioctl() - interface for application to initiate DMA
 *
 * Provides interface to the application code to start DMA read/write.
 * DMA specific info is passed by user space.
 *
 * on success it returns no of bytes DMAed.
 *
 * TODO: Do copy to/from user space for data passed.
 */
static long ti81xx_ep_dma_ioctl(struct file *file,
		unsigned int cmd, unsigned long arg)
{

	switch (cmd) {
	case TI81XX_EDMA_WRITE:
	{
		struct dma_info *dma = (struct dma_info *) arg;
		int result = -1;
		u32 bytes_transfered = (acnt * bcnt * ccnt);
		memcpy(dmabufsrcep, dma->user_buf, bytes_transfered);
		dmaphysdestrm = (dma_addr_t)dma->dest;
		result = ti81xx_edma_dma_tx(acnt, bcnt, ccnt, mode, 0);

		if (0 == result)
			return  bytes_transfered;

		return result;
	}

	break;

	case TI81XX_EDMA_WRITEM:
	{
		struct dma_info *dma = (struct dma_info *) arg;
		int result = -1;
		u32 bytes_transfered = (acnt * bcnt * ccnt);
		dmaphysdestrm = (dma_addr_t)dma->dest;
		result = ti81xx_edma_dma_tx(acnt, bcnt, ccnt, mode, 0);

		if (0 == result)
			return  bytes_transfered;

		return result;
	}
	break;

	case TI81XX_EDMA_READ:
	{
		struct dma_info *dma = (struct dma_info *) arg;
		int result = -1;
		u32 bytes_transfered = (acnt * bcnt * ccnt);
		dmaphyssrcrm = (dma_addr_t)dma->src;
		result = ti81xx_edma_dma_rx(acnt, bcnt, ccnt, mode, 0);
		if (0 == result) {
			memcpy(dma->user_buf, dmabufdestep, bytes_transfered);
			return  bytes_transfered;
		}
		return result;
	}

	break;

	case TI81XX_EDMA_READM:
	{
		struct dma_info *dma = (struct dma_info *) arg;
		int result = -1;
		u32 bytes_transfered = (acnt * bcnt * ccnt);
		dmaphyssrcrm = (dma_addr_t)dma->src;
		result = ti81xx_edma_dma_rx(acnt, bcnt, ccnt, mode, 0);
		if (0 == result)
			return  bytes_transfered;

		return result;
	}

	break;


	case TI81XX_EDMA_SET_CNT:
	{
		struct dma_cnt_conf *cnt = (struct dma_cnt_conf *)arg;
		acnt = cnt->acnt;
		bcnt = cnt->bcnt;
		ccnt = cnt->ccnt;
		mode = cnt->mode;
		//printk("acnt[0x%08x]bcnt[0x%08x]bcnt[0x%08x]mode[%d]\n",acnt,bcnt,bcnt,mode);
		return 0;
	}

	break;

	case TI81XX_EDMA_SET_BUF_INFO:
	{
		struct dma_buf_info *buf = (struct dma_buf_info *)arg;
		dmaphyssrcep = buf->send_buf;
		dmaphysdestep = buf->recv_buf;

		dmabufsrcep = (char *) ioremap_nocache(dmaphyssrcep, buf->size);
		if (!dmabufsrcep) {
			pr_err(DRIVER_NAME
				": ioremap failed for EDMA transmit buffer\n");
			return -1;
		}

		dmabufdestep = (char *)ioremap_nocache(dmaphysdestep,
							buf->size);
		if (!dmabufdestep) {
			pr_err(DRIVER_NAME
				": ioremap failed for EDMA recv buffer\n");
			iounmap((void *) dmabufsrcep);
			dmabufsrcep = NULL;
			return -1;
		}

		return 0;
	}
	break;

	case TI81XX_EDMA_WRITE_DIRECT:
	{
		struct dma_info *dma = (struct dma_info *) arg;
		dma_addr_t physsrc;
		dma_addr_t physdest;
		int result = -1;
		u32 bytes_transfered = (acnt * bcnt * ccnt);
		unsigned long start = 0,end = 0,temp = 0;
		int i;	
		//printk("bytes_transfered = 0x%08x\n",bytes_transfered);
		start = jiffies;
		
		physsrc = (dma_addr_t)dma->src;
		physdest = (dma_addr_t)dma->dest;
		
		result = ti81xx_edma_dma_tx_direct(acnt, bcnt, ccnt, mode, 0, physsrc, physdest);
		
		end = jiffies;
		temp = end -start;
		//printk("finished:time:%lld,%lld,[%lld]\n",start,end,temp);
		
		if (0 == result)
			return	bytes_transfered;

		return result;
	}

	break;

	case TI81XX_EDMA_READ_DIRECT:
	{
		struct dma_info *dma = (struct dma_info *) arg;
		dma_addr_t physsrc;
		dma_addr_t physdest;
		int result = -1;
		u32 bytes_transfered = (acnt * bcnt * ccnt);
		//printk("%s[%d]bytes_transfered = 0x%08x\n",__FUNCTION__,__LINE__,bytes_transfered);
		physsrc = (dma_addr_t)dma->src;
		physdest = (dma_addr_t)dma->dest;
		result = ti81xx_edma_dma_rx_direct(acnt, bcnt, ccnt, mode, 0, physsrc, physdest);
		//printk("%s[%d]result=%d\n",__FUNCTION__,__LINE__,result);
		if (0 == result) {
			return  bytes_transfered;
		}
		return result;
	}
	break;

	case KS2_EDMA_WRITE_DIRECT:
	{
		ks2_edma_info *edma_info = NULL;
		dma_addr_t physsrc;
		dma_addr_t physdest;
		int result = -1;
		u32 bytes_transfered;
		unsigned long start = 0,end = 0,temp = 0;
		int acnt,bcnt,ccnt,mode;

		edma_info =  (ks2_edma_info *)((void *)arg);
		
		acnt = edma_info->cnt.acnt;
		bcnt = edma_info->cnt.bcnt;
		ccnt = edma_info->cnt.ccnt;
		mode = edma_info->cnt.mode;
		physsrc = (dma_addr_t)edma_info->edma.src;
		physdest = (dma_addr_t)edma_info->edma.dest;
		bytes_transfered = (acnt * bcnt * ccnt);

		
		result = ti81xx_edma_dma_tx_direct(acnt, bcnt, ccnt, mode, 0, physsrc, physdest);
		
		//end = jiffies;
		//temp = end -start;
		//printk("finished:time:%lld,%lld,[%lld]\n",start,end,temp);
		
		if (0 == result)
			return  bytes_transfered;

		return result;
	}

	break;

	case KS2_EDMA_READ_DIRECT:
	{
		ks2_edma_info *edma_info = NULL;
		dma_addr_t physsrc;
		dma_addr_t physdest;
		int result = -1;
		u32 bytes_transfered;
		int acnt,bcnt,ccnt,mode,core;
		
		//printk("bytes_transfered = 0x%08x\n",bytes_transfered);
		//start = jiffies;
		
		edma_info =  (ks2_edma_info *)((void *)arg);
		acnt = edma_info->cnt.acnt;
		bcnt = edma_info->cnt.bcnt;
		ccnt = edma_info->cnt.ccnt;
		mode = edma_info->cnt.mode;
		core = edma_info->cnt.cord_idx;

		
		physsrc = (dma_addr_t)edma_info->edma.src;
		physdest = (dma_addr_t)edma_info->edma.dest;
		bytes_transfered = (acnt * bcnt * ccnt);
		
		result = ti81xx_edma_dma_rx_direct(acnt, bcnt, ccnt, core, 0, physsrc, physdest);
		//printk("%s[%d]result=%d\n",__FUNCTION__,__LINE__,result);
		if (0 == result) {
			return  bytes_transfered;
		}
		return result;
	}
	break;


	default:
	{
		printk("%s[%d]cmd = 0x%x\n",__FUNCTION__,__LINE__,cmd);
		return -1;
	}

	}
}

static int ti81xx_ep_dma_mmap(struct file *filp, struct vm_area_struct *vma)
{
	int ret = -EINVAL;
	unsigned long sz = vma->vm_end - vma->vm_start;
	unsigned int addr = (unsigned int)vma->vm_pgoff << PAGE_SHIFT;
	pr_debug(DRIVER_NAME ":Mapping %#lx bytes from address %#x\n",
			sz, addr);
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	ret = remap_pfn_range(vma, vma->vm_start,
					vma->vm_pgoff,
						sz, vma->vm_page_prot);
	return ret;
}

/**
 * ti81xx_ep_dma_fops - declares supported file access functions
 */

static const struct file_operations ti81xx_ep_dma_fops = {
	.owner          = THIS_MODULE,
	.unlocked_ioctl = ti81xx_ep_dma_ioctl,
	.mmap           = ti81xx_ep_dma_mmap,
};


/*
 * init part of the module
 */


static int __init edma_init(void)
{
	int ret = 0;
	int i = 0;
	unsigned int dma_ch = 0;

	ret = alloc_chrdev_region(&ti81xx_ep_dma_dev, DEV_MINOR, DEVICENO,
							TI81XX_DMA_MODFILE);
	if (ret < 0) {

		pr_err(DRIVER_NAME ": could not allocate the character driver");
		return -1;
	}

	cdev_init(&ti81xx_ep_dma_cdev, &ti81xx_ep_dma_fops);
	ret = cdev_add(&ti81xx_ep_dma_cdev, ti81xx_ep_dma_dev, DEVICENO);
	if (ret < 0) {
		pr_err(DRIVER_NAME ": cdev add failed");
		unregister_chrdev_region(ti81xx_ep_dma_dev, DEVICENO);
		return -1;
	}

	ti81xx_dma_class = class_create(THIS_MODULE, TI81XX_DMA_MODFILE);
	if (!ti81xx_dma_class) {
		pr_err(DRIVER_NAME ":failed to add device to sys fs");
		goto ERROR;
	}

	device_create(ti81xx_dma_class, NULL, ti81xx_ep_dma_dev, NULL,
							TI81XX_DMA_MODFILE);

	for (i = 0; i < MAX_CORE_NUM; i++)
	{
		init_waitqueue_head(&write_queue[i]);
		init_waitqueue_head(&read_queue[i]);
	}
	
	init_waitqueue_head(&read_queue0);
	init_waitqueue_head(&read_queue1);
	init_waitqueue_head(&read_queue2);
	init_waitqueue_head(&read_queue3);

	sema_init(&read_sema0, 0);
	sema_init(&read_sema1, 0);
	init_completion(&read_completion0);
	init_completion(&read_completion1);
	init_completion(&read_completion2);
	init_completion(&read_completion3);





	//for(i = 0; i < 320; i++)
	//{
	//	dma_ch = edma_alloc_channel(EDMA_CHANNEL_ANY, callback2, NULL, EVENTQ_DEFAULT);
	//	printk("%s[%d]dma_ch[%d] = 0x%x\n",__FUNCTION__,__LINE__,i,dma_ch);
	//}

	return 0;
ERROR:
	iounmap((void *) dmabufsrcep);
	iounmap((void *) dmabufdestep);
	cdev_del(&ti81xx_ep_dma_cdev);
	unregister_chrdev_region(ti81xx_ep_dma_dev, DEVICENO);

	return -1;
}


/*
 * exit part of the module
 */

void edma_exit(void)
{

	device_destroy(ti81xx_dma_class, ti81xx_ep_dma_dev);
	class_destroy(ti81xx_dma_class);
	iounmap((void *)dmabufdestep);
	iounmap((void *)dmabufsrcep);
	cdev_del(&ti81xx_ep_dma_cdev);
	unregister_chrdev_region(ti81xx_ep_dma_dev, DEVICENO);

	pr_info(DRIVER_NAME ": Exiting TI81XX EDMA3 Module\n");
}


/* DMA TX */

int ti81xx_edma_dma_tx(int acnt, int bcnt, int ccnt, int sync_mode,
							int event_queue)
{
	int result = 0;


	return result;
}


int ti81xx_edma_dma_rx(int acnt, int bcnt, int ccnt, int sync_mode,
							int event_queue)
{
	int result = 0;


	return result;
}








/* DMA TX */

int ti81xx_edma_dma_tx_direct(int acnt, int bcnt, int ccnt, int sync_mode,
							int event_queue,dma_addr_t src,dma_addr_t dest)
{
	int result = 0;
	unsigned int dma_ch = 0;
	int i;

	unsigned int numenabled = 0;
	unsigned int BRCnt = 0;
	int srcbidx = 0;
	int desbidx = 0;
	int srccidx = 0;
	int descidx = 0;
	struct edmacc_param param_set;
	unsigned ctlr = 0;


	/* Set B count reload as B count. */
	BRCnt = bcnt;

	/* Setting up the SRC/DES Index */
	srcbidx = acnt;
	desbidx = acnt;

	srccidx = 0;//acnt;
	descidx = 0;//acnt;

	dma_ch = edma_alloc_channel(EDMA_CHANNEL_ANY, callback4,
						NULL, EVENTQ_DEFAULT);
	pr_debug(DRIVER_NAME ": dma_ch %d is allocated\n", dma_ch);
	//printk("dma_ch %d is allocated\n",__FUNCTION__,__LINE__,dma_ch);
	
	printk("EMDA TX:src[0x%08x]dest[0x%08x]acnt[0x%08x]dma_ch[0x%x]\n",src,dest,acnt,dma_ch);

	if (dma_ch < 0) {
		pr_debug(DRIVER_NAME ": dma channel allocation failed");
		return dma_ch;
	}
	ctlr = EDMA_CTLR(dma_ch);

	edma_set_src(dma_ch, (unsigned long)(src), INCR, W8BIT);

	edma_set_dest(dma_ch, (unsigned long)(dest), INCR, W8BIT);

	edma_set_src_index(dma_ch, srcbidx, srccidx);

	edma_set_dest_index(dma_ch, desbidx, descidx);

	edma_set_transfer_params(dma_ch, acnt, bcnt, ccnt, BRCnt, sync_mode);

	/* Enable the Interrupts on Channel 1 */
	edma_read_slot(dma_ch, &param_set);

//	param_set.opt |= (1 << ITCCHEN_SHIFT);
//	param_set.opt |= (1 << TCCHEN_SHIFT);

	param_set.opt |= (1 << ITCINTEN_SHIFT);	//y
	param_set.opt |= (1 << TCINTEN_SHIFT);
	param_set.opt |= EDMA_TCC(EDMA_CHAN_SLOT(dma_ch));

//	printk("%s[%d]param_set.opt = 0x%08x\n",__FUNCTION__,__LINE__,param_set.opt);
//	printk("%s[%d]param_set.src = 0x%08x\n",__FUNCTION__,__LINE__,param_set.src);
//	printk("%s[%d]param_set.a_b_cnt = 0x%08x\n",__FUNCTION__,__LINE__,param_set.a_b_cnt);
//	printk("%s[%d]param_set.dst = 0x%08x\n",__FUNCTION__,__LINE__,param_set.dst);
//	printk("%s[%d]param_set.src_dst_bidx = 0x%08x\n",__FUNCTION__,__LINE__,param_set.src_dst_bidx);
//	printk("%s[%d]param_set.link_bcntrld = 0x%08x\n",__FUNCTION__,__LINE__,param_set.link_bcntrld);
//	printk("%s[%d]param_set.src_dst_cidx = 0x%08x\n",__FUNCTION__,__LINE__,param_set.src_dst_cidx);
//	printk("%s[%d]param_set.ccnt = 0x%08x\n",__FUNCTION__,__LINE__,param_set.ccnt);



	edma_write_slot(dma_ch, &param_set);
	if (sync_mode == ASYNC)
		numenabled = bcnt * ccnt;
	else
		numenabled = ccnt;


	for (i = 0; i < numenabled; i++) {
		irqraised1[ctlr] = 0;
		//clear_interrupt(dma_ch);
		/* Now enable the transfer as many times as calculated above*/
		result = edma_start(dma_ch);
		if (result != 0) {
			pr_err(DRIVER_NAME ": edma_start failed");
			break;
		}
		
//		printk("%s[%d]numenabled = %d,i = %d\n",__FUNCTION__,__LINE__,numenabled,i);
//		//wait_transfer(dma_ch);
//		printk("%s[%d]\n",__FUNCTION__,__LINE__);
		/* Wait for the Completion ISR. */
		wait_event_interruptible(write_queue[ctlr], irqraised1[ctlr] == 1);
//		printk("%s[%d]\n",__FUNCTION__,__LINE__);
		/* Check the status of the completed transfer */
		if (irqraised1[ctlr] < 0) {
			/* Some error occured, break from the FOR loop. */
			pr_err(DRIVER_NAME ": Event Miss Occured!!!\n");
			break;
		}
	}

	if (0 == result) {
		edma_stop(dma_ch);
		edma_clean_channel(dma_ch);
		edma_free_slot(dma_ch);
		edma_free_channel(dma_ch);
		pr_debug(DRIVER_NAME ": channel cleanup done");
	}

	pr_debug(DRIVER_NAME ": TI81XX EDMA TX transfer complete\n");

	return result;
}
EXPORT_SYMBOL(ti81xx_edma_dma_tx_direct);


int ti81xx_edma_dma_rx_direct(int acnt, int bcnt, int ccnt, int core_idx,
							int event_queue,dma_addr_t src,dma_addr_t dest)
{
	int result = 0;
	unsigned int dma_ch = 0;
	int i;

	unsigned int numenabled = 0;
	unsigned int BRCnt = 0;
	int srcbidx = 0;
	int desbidx = 0;
	int srccidx = 0;
	int descidx = 0;
	struct edmacc_param param_set;
	unsigned ctlr = 0;

	/* Set B count reload as B count. */
	BRCnt = bcnt;

	/* Setting up the SRC/DES Index */
	srcbidx = acnt;
	desbidx = acnt;

	srccidx = acnt * bcnt;
	descidx = acnt * bcnt;
	
	dma_ch = edma_alloc_channel_for_core(EDMA_CHANNEL_ANY, core_idx, callback3,
							NULL, EVENTQ_DEFAULT);
	
	//printk("%s[%d]dma_ch = 0x%x\n",__FUNCTION__,__LINE__,dma_ch);
	
	pr_debug(DRIVER_NAME ": dma_ch %d is allocated\n", dma_ch);
	if (dma_ch < 0) {
		pr_debug(DRIVER_NAME ": dma channel allocation failed");
		pr_err("rx channel fail\n");
		return dma_ch;
	}

	
	ctlr = EDMA_CTLR(dma_ch);

	//printk("core_idx:%d,%d,0x%x\n",core_idx,ctlr,dma_ch);

	
	edma_set_src(dma_ch, (unsigned long)(src), INCR, W8BIT);

	edma_set_dest(dma_ch, (unsigned long)(dest), INCR, W8BIT);

	edma_set_src_index(dma_ch, srcbidx, srccidx);

	edma_set_dest_index(dma_ch, desbidx, descidx);

	edma_set_transfer_params(dma_ch, acnt, bcnt, ccnt, BRCnt, ABSYNC);

	/* Enable the Interrupts on Channel 1 */
	edma_read_slot(dma_ch, &param_set);
	param_set.opt |= (1 << ITCINTEN_SHIFT);
	param_set.opt |= (1 << TCINTEN_SHIFT);
	param_set.opt |= EDMA_TCC(EDMA_CHAN_SLOT(dma_ch));
	edma_write_slot(dma_ch, &param_set);
	
	//if (sync_mode == ASYNC)
	//	numenabled = bcnt * ccnt;
	//else
	numenabled = ccnt;
	//printk("%s[%d]numenabled=%d\n",__FUNCTION__,__LINE__,numenabled);
	
	for (i = 0; i < numenabled; i++) {
		irqraised2[ctlr] = 0;
		
		/* Now enable the transfer as many times as calculated above*/
		result = edma_start(dma_ch);
		if (result != 0) {
			pr_err(DRIVER_NAME ": edma_start failed");
			break;
		}

		/* Wait for the Completion ISR. */
		wait_event_interruptible(read_queue[ctlr], irqraised2[ctlr] == 1);

		/* Check the status of the completed transfer */
		if (irqraised2[ctlr] < 0) {
			/* Some error occured, break from the FOR loop. */
			pr_err(DRIVER_NAME ": Event Miss Occured!!!\n");
			break;
		}
	}
	
	if (0 == result)
	{
		edma_stop(dma_ch);
		edma_clean_channel(dma_ch);
		edma_free_slot(dma_ch);
		//edma_free_channel(dma_ch);
		pr_debug(DRIVER_NAME ": channel cleanup done");
	}

	pr_debug(DRIVER_NAME ": TI81XX EDMA RX transfer complete\n");

	return result;
}

module_init(edma_init);
module_exit(edma_exit);

MODULE_AUTHOR("Texas Instruments");
MODULE_LICENSE("GPL");
