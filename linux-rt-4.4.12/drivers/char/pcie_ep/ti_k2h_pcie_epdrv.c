/**
 * TI_K2H PCIe EP driver.
 *
 * Copyright (C) 2011 Texas Instruments, Incorporated
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
 
#include <linux/init.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/fcntl.h>
#include <linux/uaccess.h>
#include <linux/ioport.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/semaphore.h>
#include <linux/spinlock.h>

#include <asm/irq.h>
#include <linux/mm.h>
#include <linux/ioport.h>
#include <linux/jiffies.h>
#include <linux/list.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/semaphore.h>
#include <linux/sched.h>
#include <linux/moduleparam.h>
#include <asm/atomic.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/clk.h>
#include <asm/signal.h>
#include <linux/kthread.h>

#include "ti_k2h_pcie_epdrv.h"
#include "pcie_keystone_regs.h"
#include "pcie_keystone_drv.h"
#include "ks2_pcie_reg.h"
#include "data_tx_list.h"

#define SUPPORT_PCIE_MSI
#define SUPPORT_ASYNC
//#define SUPPORT_EDMA

struct serdes_config {
	u32 reg;
	u32 val;
	u32 mask;
};
struct ks2_int_info {
	struct work_struct tx_wq;
	struct resource inbound_dma_res[5];
	int legacy_irqs[MAX_LEGACY_HOST_IRQS];
	int msi_irqs[MAX_MSI_HOST_IRQS];
	int error_irq;
	uint32_t dest_addr;
	uint32_t src_addr;
	uint16_t size;
};
typedef struct packet_info {
	uint32_t	dest_addr;
	uint32_t	src_addr;
	uint16_t	size;
	uint16_t 	flag;
} packet_info_t;


static struct ks2_int_info pcie_int_info;

static packet_info_t packet_array[5120];
static uint16_t packet_array_write_index = 0;
static uint16_t packet_array_read_index = 0;

/**
 * ti_k2h_outb_info: this structure represents current outbound mappings info.
 * @list: to form a list using standard list methods in kernel
 * @ob_offset_hi: higher 32 bit address, as send in outbound setup
 * @ob_offset_idx: lower 32 bit address, as send in outbound setup
 * @start_region: region no. from which this mapping start
 * @regions: represents no of regions used
 * @size_req: size requested by this mapping.
 *
 * Each outbound related request, if successfull,  become part of a list
 * in which every node is of ti81xx_outb_info type.
 *
 * this information is helpfull when application request for clearing outbound
 * mapping done earlier.
 *
 * It is the responsibility of application to clear unused mapping.
 *
 */


struct ti_k2h_outb_info {
	struct list_head list;
	u32 ob_offset_hi;
	u32 ob_offset_idx;
	u32 start_region;
	u32 regions;
	u32 size_req;
};

struct keystone_pcie_info {
	void __iomem *reg_cfg_virt;
	/* PCIE resources */
	int num_mem_resources;
	struct resource cfg_resource;
	struct resource	mem_resource;
	struct resource	io_resource;
	struct resource inbound_dma_res;
	/* legacy Host irqs */
	int num_legacy_irqs;
	int legacy_irqs[MAX_LEGACY_HOST_IRQS];
	struct irq_domain *legacy_irqd;
	int virqs[MAX_LEGACY_HOST_IRQS];
	/* MSI IRQs */
	int num_msi_host_irqs;
	int msi_host_irqs[MAX_MSI_HOST_IRQS];
	int num_msi_irqs;
	int msi_virq_base;
	struct irq_domain *msi_irqd;
	int error_irq;
	/* platform customization */
	struct keystone_pcie_pdata *pdata;
};

/* keystone pcie pdata configurations */
struct keystone_pcie_pdata {
	int (*setup)(void *pdata, struct device_node *);
	int en_link_train;
};

static DEFINE_MUTEX(tx_mutex);
static struct task_struct *tx_thread;
static struct task_struct *debug_thread;

static struct semaphore tx_queue_sema;
spinlock_t	tx_lock;

u32 ti_k2h_ep_mem_start;
u32 ti_k2h_ep_mem_size;

static struct keystone_pcie_info *ep_info;


dev_t ti_k2h_ep_pcie_dev;
static struct cdev ti_k2h_ep_pcie_cdev;
static u32 reg_vir = 0, reg_mem,ib_vir,mem_reg_vir = 0,ks2reg_vir = 0;
static void __iomem *gpio_reg_va; 

static u16 device_id;
static struct resource *res_mem,*ib_res_mem;
/* 6MB : default value of size to be reserved for
 * application buffers and MGMT area
 */
/* !@@@X static unsigned int resv_size = (6 * 1024 * 1024);*/
static struct class *ti_k2h_pci_class;

/* status global variable.
 * represent status of outbound regions
 * each bit of this integer is correspond to a outbound region
 * if bit set-- free
 * else-- in use
 */

static u32 status_out;
static atomic_t         irq_raised = ATOMIC_INIT(0);
/* !@@@X static unsigned int *mgmt_area_start;*/
/* !@@@X static unsigned int *mgmt_area_remap;*/
static struct list_head outb_req;
wait_queue_head_t readq;
static struct semaphore sem_poll;
static atomic_t          counter = ATOMIC_INIT(0);

static struct fasync_struct *async; //声明fasync_struct

/* passed at module load time, default value is 6M */
/* !@@@X module_param(resv_size, uint , S_IRUGO);*/

/**
 * declaration of functions used
 */

static int ti_k2h_set_inbound(struct ti_k2h_inb_window *in);
static u32  ti_k2h_check_regions(unsigned int  regions);
static u32 ti_k2h_check_req_status(struct ti_k2h_outb_region *out,
							u32 ob_size);
static int ti_k2h_set_out_region(struct ti_k2h_outb_region *out);
static int ti_k2h_clr_outb_map(struct ti_k2h_outb_region *out);
static int ti_k2h_pciess_register_access(void __user *arg);
static int ti_k2h_pciess_Bar1_access(void __user *arg);

#if 0
/*** this will be supported only if driver is built staticaly in kernel ******/
static int ti816x_pcie_fault(unsigned long addr, unsigned int fsr,
							struct pt_regs *regs);
#endif
static int ti_k2h_send_msi(struct ti_k2h_msi_bar *bar);
static int ti_k2h_ep_pcie_open(struct inode *inode, struct file *file);
static long ti_k2h_ep_pcie_ioctl(struct file *file, unsigned int cmd,
							unsigned long arg);
static unsigned int ti_k2h_pcie_poll(struct file *filp, poll_table *wait);
static int ti_k2h_ep_pcie_mmap(struct file *filp, struct vm_area_struct *vma);
static int ti_k2h_ep_pcie_close(struct inode *inode, struct file *filp);
static int ti_k2h_setup_msi(void);
static int query_bar_info(void __user *arg);
static irqreturn_t ti_k2h_ep_pcie_handle(int irq, void *dev);

static void tx_wq_func(struct work_struct *work);




//extern int ti81xx_edma_dma_tx_direct(int acnt, int bcnt, int ccnt, int sync_mode,
//							int event_queue,dma_addr_t src,dma_addr_t dest);

#define TX_RX_BUF_SZ		40
#define IB_BUFFER_DEFAULT_SIZE		SZ_32K
#define IB_BUFFER_ALIGN_SIZE		0x100
#define IB_BUFFER_ALIGN_MASK		(IB_BUFFER_ALIGN_SIZE - 1)

u32 *ib_buffer;



PCIE_Memory_Region memory_regions[]=
{
	{0x90000000, 32*1024*1024}, //DDR3A
#ifdef DDRB_TEST
	{0x70000000, 32*1024*1024},  //DDR3B
#else
	{0xA0000000, 32*1024*1024},  //DDR3A
#endif
	{0x0C100000, 1*1024*1024}, 	//MSMC_RAM
	{0x10820000, 256*1024} 	//LL2
};

#define PCIE_RC_BAR0_ADDRESS 		0x01000000 	//should >=0x400000???
unsigned long long EP_OB_PCIE_address[]=
{//EP OB configuration should be done manually
	PCIE_RC_BAR0_ADDRESS,   
	0x0000000080000000,
	0x0000000090000000
	//to do
};


static int keystone_get_l2_addr(unsigned offset)
{
	uint32_t value;
	
	value = readl(gpio_reg_va + 0x00);
	printk("Get GPIO PID:0x%08x\n",value);
	
	value = readl(gpio_reg_va + 0x20);
	printk("Get GPIO InData:0x%08x\n",value);
	
	return value;
}

static int ti_k2h_send_intx_2_pc(void)
{
	__raw_writel(1, reg_vir + LEGACY_N_IRQ_ENABLE_SET(0));
	__raw_writel(1, reg_vir + EP_IRQ_SET);
	return 0;
}


static void tx_wq_func(struct work_struct *work)
{
	struct ks2_int_info *info = container_of(work, struct ks2_int_info, tx_wq);
	uint32_t pcie_src_addr = 0,pcie_dest_addr = 0;
	uint16_t data_size = 0;
	raw_packet_data_t 	p_output_image;
	int ret;
	
//	printk("tx_wq_func in\n");
	
//	spin_lock_bh(&tx_lock);

	if(packet_array[packet_array_read_index].flag)
	{
		pcie_src_addr = packet_array[packet_array_read_index].src_addr;
		pcie_dest_addr = packet_array[packet_array_read_index].dest_addr;
		data_size = packet_array[packet_array_read_index].size;
		packet_array[packet_array_read_index].flag = 0;
		packet_array_read_index ++;
	}
	if(packet_array_read_index >= 5120)
	packet_array_read_index = 0;
	
//	pcie_src_addr = info->src_addr;
//	pcie_dest_addr = info->dest_addr;
//	data_size = info->size;

//	spin_unlock_bh(&tx_lock);
	
	ret = queue_empty();
	//if (ret)
	//printk("The queue is empty\n");
	
//	printk("enqueue:src[0x%08x]dest[0x%08x]size[0x%08x]\n",pcie_src_addr,pcie_dest_addr,data_size);
	
	if ((data_size != 0) && (pcie_src_addr != 0) && (pcie_dest_addr != 0))
	{
		enqueue_txData(pcie_dest_addr, pcie_src_addr, data_size);

		if(ret == 1)
		up(&tx_queue_sema);
	}
	//printk("tx_wq_func in\n");

}

#ifdef SUPPORT_EDMA

static int tx_thread_func(void *data)
{
	uint32_t pcie_src_addr,pcie_dest_addr;
	uint16_t data_size;
	
	raw_packet_data_t 	p_output_image;
	int ret;
	int add_value = 0;
	
	printk("tx_thread_func In\n");
	
	while (1) {
		//printk("wait semaphore In\n");
		if (_down_sema(&tx_queue_sema) == 0)
		{
			printk("Get semaphore fail\n");
			break;
		}
		
		//printk("%d:Get semaphore ok\n",__LINE__);
		
		_next:
		ret =  dequeue_txData(&p_output_image);
		if(ret == 0)
		{
			//printk("The queue is empty!!!!!!!!!!!\n");
			continue;
		}

		
		pcie_src_addr = p_output_image.src_addr;
		pcie_dest_addr = p_output_image.dest_addr;
		data_size = p_output_image.size;
		
		//printk("dequeue:src[0x%08x]dest[0x%08x]size[0x%08x]\n",pcie_src_addr,pcie_dest_addr,data_size);
		#if 0
		if ((data_size != 0) && (pcie_src_addr != 0) && (pcie_dest_addr != 0))
		{
			ret = ti81xx_edma_dma_tx_direct(data_size, 1, 1, 0, 0, pcie_src_addr, pcie_dest_addr);
			add_value ++;
			
			if(ret == 0 && add_value == 100)
			{
				add_value = 0;
				ti_k2h_send_intx_2_pc();
			}
		}
		#endif
		goto _next;
	}

	return 0;
}
#endif

static int debug_thread_func(void *data)
{
	u32 value = 0;
	
	printk("tx_thread_func In\n");
	value = readl(gpio_reg_va + 0x10) & 0xBFFFFFFF;
	writel(value,gpio_reg_va + 0x10);

	while (1) {
		writel(0x40000000,gpio_reg_va + 0x18);
		msleep(500);
		writel(0x40000000,gpio_reg_va + 0x1C);
		msleep(500);
	}

	return 0;
}


void keystone_pcie_send_data(void)
{
	int i;
	u32 src_buf[TX_RX_BUF_SZ];
	void __iomem *pcie_data_regs;

	for (i = 0; i < TX_RX_BUF_SZ; i++)
		src_buf[i] = i;

	pcie_data_regs = ioremap(0x50000000, 0x1000);

	printk( ": RC sending data (%08x) to EP ...",
		(u32)pcie_data_regs);
	for (i = 0; i < TX_RX_BUF_SZ; i++)
		__raw_writel(src_buf[i], pcie_data_regs + i*4);

	/* Signal end of transmit to EP */
	__raw_writel(1, pcie_data_regs + TX_RX_BUF_SZ*4);
	printk( ": RC send data to EP done.\n");
}
EXPORT_SYMBOL(keystone_pcie_send_data);

void keystone_pcie_send_receive_data(void)
{
	int i;
	u32 src_buf[TX_RX_BUF_SZ];
	void __iomem *pcie_data_regs;

	for (i = 0; i < TX_RX_BUF_SZ; i++)
		src_buf[i] = i;

	pcie_data_regs = ioremap(0x50000000, 0x1000);

	printk( ": RC sending data to EP ...\n");
	for (i = 0; i < TX_RX_BUF_SZ; i++)
		__raw_writel(src_buf[i], pcie_data_regs + i*4);

	/* Signal end of transmit to EP */
	__raw_writel(1, pcie_data_regs + TX_RX_BUF_SZ*4);
	printk( ": RC send data to EP done.\n");

	printk( ": RC receiving data @ %08x - %08x from EP ...\n",
		(u32)ib_buffer, (u32)&(ib_buffer[TX_RX_BUF_SZ]));

	msleep(1000);

	while (ib_buffer[TX_RX_BUF_SZ] != 1)
		i = 0;

	printk( ": RC received data from EP done.\n");

	printk( ": RC validating received data ...\n");
	for (i = 0; i < TX_RX_BUF_SZ; i++) {
		if (ib_buffer[i] != src_buf[i]) {
			printk(
				": Error in receiving %d-th entry ", i);
			return;
		}
	}
	printk( ": RC received data validated\n");
}
EXPORT_SYMBOL(keystone_pcie_send_receive_data);


static void mask_legacy_irq(void __iomem *reg_virt, int i)
{
	__raw_writel(0x1, reg_virt + IRQ_ENABLE_CLR + (i << 4));
}

static void unmask_legacy_irq(void __iomem *reg_virt, int i)
{
	__raw_writel(0x1, reg_virt + IRQ_ENABLE_SET + (i << 4));
}

/**
 * keystone_legacy_irq_handler() - Handle legacy interrupt
 * @irq: IRQ line for legacy interrupts
 * @desc: Pointer to irq descriptor
 *
 * Traverse through pending legacy interrupts and invoke handler for each. Also
 * takes care of interrupt controller level mask/ack operation.
 */
static void keystone_legacy_irq_handler(unsigned int irq, struct irq_desc *desc)
{
	struct keystone_pcie_info *info = irq_desc_get_handler_data(desc);
	u32 irq_offset = irq - info->legacy_irqs[0], pending;
	struct irq_chip *chip = irq_desc_get_chip(desc);
	int virq;
	//u32 testcode;
	

	printk("Handling legacy irq %d\n", irq);

	//testcode = __raw_readl(reg_vir + LOCAL_CONFIG_OFFSET +VENDOR_DEVICE_ID);
	//printk("%s:the id is 0x%08x\n",__FUNCTION__,testcode);



	/*
	 * The chained irq handler installation would have replaced normal
	 * interrupt driver handler so we need to take care of mask/unmask and
	 * ack operation.
	 */
	chip->irq_mask(&desc->irq_data);
	if (chip->irq_ack)
		chip->irq_ack(&desc->irq_data);

	pending = __raw_readl(info->reg_cfg_virt +
			IRQ_STATUS + (irq_offset << 4));

	printk("IRQ_STATUS = 0x%x\n",pending);
	
	if (BIT(0) & pending) {
		virq = irq_linear_revmap(info->legacy_irqd, irq_offset);
		pr_info(DRIVER_NAME
			": irq: irq_offset %d, virq %d\n", irq_offset, virq);
		generic_handle_irq(virq);
	}

	/* EOI the INTx interrupt */
	__raw_writel(irq_offset, info->reg_cfg_virt + IRQ_EOI);

	if (chip->irq_eoi)
		chip->irq_eoi(&desc->irq_data);
	
	chip->irq_unmask(&desc->irq_data);
}
static void keystone_legacy_irq_handler_1(int irq, void *dev)
{
	uint32_t i, dma_failure_flag = 0, counter = 0;
	uint32_t status = __raw_readl(reg_vir + EP_IRQ_STATUS);
	
	u32 irq_offset = irq - 58;

	printk("%s[%d]status = %d\n",__FUNCTION__,__LINE__,status);
	
	if (status == 1) {
		printk("Interrupt %d received from DSP\n", irq);
		
		//if(async)
		kill_fasync(&async, SIGIO, POLL_IN);  //向打开设备文件的进程发出SIGIO信号  

		__raw_writel(1, reg_vir + EP_IRQ_CLR);
		__raw_writel(irq_offset, reg_vir + IRQ_EOI);
		return IRQ_HANDLED;
	}
	return IRQ_NONE;
}


static void ack_irq(struct irq_data *d)
{

}

static void mask_irq(struct irq_data *d)
{

}

static void unmask_irq(struct irq_data *d)
{

}

static struct irq_chip keystone_legacy_irq_chip = {
	.name = "PCIe-LEGACY-IRQ",
	.irq_ack = ack_irq,
	.irq_mask = mask_irq,
	.irq_unmask = unmask_irq,
};

static DECLARE_BITMAP(msi_irq_bits, MAX_MSI_IRQS);
static void ack_msi(struct irq_data *d)
{
	struct keystone_pcie_info *info = irq_data_get_irq_chip_data(d);
	u32 offset, reg_offset, bit_pos;
	unsigned int irq = d->irq;

	//offset = irq - irq_linear_revmap(info->msi_irqd, 0);
	offset = 0;

	reg_offset = offset % 8;
	bit_pos = offset >> 3;

	__raw_writel(BIT(bit_pos),
		info->reg_cfg_virt + MSI0_IRQ_STATUS + (reg_offset << 4));

	__raw_writel(reg_offset + MSI_IRQ_OFFSET, info->reg_cfg_virt + IRQ_EOI);
}

static void mask_msi(struct irq_data *d)
{
	printk("%s[%d]\n",__FUNCTION__,__LINE__);
	struct keystone_pcie_info *info = irq_data_get_irq_chip_data(d);
	printk("%s[%d]\n",__FUNCTION__,__LINE__);
	u32 offset, reg_offset, bit_pos;
	unsigned int irq = d->irq;

	//offset = irq - irq_linear_revmap(info->msi_irqd, 0);
	offset = 0;


	reg_offset = offset % 8;
	bit_pos = offset >> 3;

	__raw_writel(BIT(bit_pos),
		info->reg_cfg_virt + MSI0_IRQ_ENABLE_CLR + (reg_offset << 4));
}

static void unmask_msi(struct irq_data *d)
{
	struct keystone_pcie_info *info = irq_data_get_irq_chip_data(d);
	u32 offset, reg_offset, bit_pos;
	unsigned int irq = d->irq;

	//offset = irq - irq_linear_revmap(info->msi_irqd, 0);
	offset = 0;

	reg_offset = offset % 8;
	bit_pos = offset >> 3;

	__raw_writel(BIT(bit_pos),
		info->reg_cfg_virt + MSI0_IRQ_ENABLE_SET + (reg_offset << 4));
}

/*
 * TODO: Add support for mask/unmask on remote devices (mask_msi_irq and
 * unmask_msi_irq). Note that, this may not work always - requires endpoints to
 * support mask bits capability.
 */
static struct irq_chip keystone_msi_chip = {
	.name = "PCIe-MSI",
	.irq_ack = ack_msi,
	.irq_mask = mask_msi,
	.irq_unmask = unmask_msi,
};

/**
 * keystone_msi_handler() - Handle MSI interrupt
 * @irq: IRQ line for MSI interrupts
 * @desc: Pointer to irq descriptor
 *
 * Traverse through pending MSI interrupts and invoke handler for each. Also
 * takes care of interrupt controller level mask/ack operation.
 */
static void keystone_msi_handler_1(unsigned int irq, struct irq_desc *desc)
{
	struct keystone_pcie_info *info = irq_desc_get_handler_data(desc);
	u32 offset = irq - info->msi_host_irqs[0], pending, vector;
	struct irq_chip *chip = irq_desc_get_chip(desc);
	int src, virq;
	u32 testcode;

	printk("Handling MSI irq %d\n", irq);
	testcode = __raw_readl(reg_vir + LOCAL_CONFIG_OFFSET +VENDOR_DEVICE_ID);
	printk("%s:the id is 0x%08x\n",__FUNCTION__,testcode);
	
	/*
	 * The chained irq handler installation would have replaced normal
	 * interrupt driver handler so we need to take care of mask/unmask and
	 * ack operation.
	 */
	 
	printk("%s:&desc->irq_data = 0x%08x\n",__FUNCTION__,&desc->irq_data);
	
	mask_msi(&desc->irq_data);
	printk("%s[%d]\n",__FUNCTION__,__LINE__);
	ack_msi(&desc->irq_data);
	printk("%s[%d]\n",__FUNCTION__,__LINE__);
	
	pending = __raw_readl(info->reg_cfg_virt +
			MSI0_IRQ_STATUS + (offset << 4));
	
	printk("MSI0_IRQ_STATUS 0x%x\n", pending);
	
	/*
	 * MSI0, Status bit 0-3 shows vectors 0, 8, 16, 24, MSI1 status bit
	 * shows 1, 9, 17, 25 and so forth
	 */
	for (src = 0; src < 4; src++) {
		if (BIT(src) & pending) {
			vector = offset + (src << 3);
			//virq = irq_linear_revmap(info->msi_irqd, vector);
			//pr_info(DRIVER_NAME
			//	": irq: bit %d, vector %d, virq %d\n",
			//	 src, vector, virq);
			//generic_handle_irq(virq);
		}
	}

	printk("%s[%d]\n",__FUNCTION__,__LINE__);
	//if (chip->irq_eoi)
	//	chip->irq_eoi(&desc->irq_data);
	
	__raw_writel(4, reg_vir + IRQ_EOI);
	
	printk("%s[%d]\n",__FUNCTION__,__LINE__);
	unmask_msi(&desc->irq_data);
	printk("%s[%d]\n",__FUNCTION__,__LINE__);
}

#ifdef SUPPORT_EDMA
static irqreturn_t keystone_msi_handler(int irq, void *dev)
{
	struct ks2_int_info *info = dev;
	u32 testcode;
	u32 pending;
	int offset = irq - info->msi_irqs[0];
	uint32_t pcie_src_addr,pcie_dest_addr;
	uint32_t data_size;
	u32 retval = 0;
	
	int i = 0;
	
	//printk("Handling MSI irq %d,offset[%d]\n", irq,offset);
	//testcode = __raw_readl(reg_vir + LOCAL_CONFIG_OFFSET +VENDOR_DEVICE_ID);
	//printk("%s:the id is 0x%08x\n",__FUNCTION__,testcode);
	
	//mask	
	__raw_writel(1,reg_vir + MSIX_IRQ(offset) + MSI_IRQ_ENABLE_CLR);
	//printk("%s[%d]\n",__FUNCTION__,__LINE__);
	//ack
	__raw_writel(1,reg_vir + MSIX_IRQ(offset) + MSI0_IRQ_STATUS);
	__raw_writel(MSI_IRQ_OFFSET+offset, reg_vir + IRQ_EOI);

	
	//printk("%s[%d]\n",__FUNCTION__,__LINE__);
	
	pending = __raw_readl(reg_vir + MSIX_IRQ(offset)+MSI_IRQ_STATUS);
	//printk("MSI0_IRQ_STATUS 0x%x\n", pending);


	do {
		//printk("Do something!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
		__raw_writel(0xFFFFFFFF,reg_vir + MSIX_IRQ(offset)+MSI_IRQ_STATUS);
		atomic_set(&irq_raised, 1);
		atomic_inc(&counter);
		//wake_up_interruptible(&readq);

		pcie_src_addr = __raw_readl(mem_reg_vir + PCIE_SRC_ADDR(offset));
		pcie_dest_addr = __raw_readl(mem_reg_vir + PCIE_DEST_ADDR(offset));
		data_size = __raw_readl(mem_reg_vir + PCIE_SIZE(offset));
		
//		printk("irq:src[0x%08x]dest[0x%08x]size[0x%08x]offset[%d]\n",pcie_src_addr,pcie_dest_addr,data_size,offset);

		packet_array[packet_array_write_index].dest_addr = pcie_dest_addr;
		packet_array[packet_array_write_index].src_addr = pcie_src_addr;
		packet_array[packet_array_write_index].size = data_size;
		packet_array[packet_array_write_index].flag = 1;
		packet_array_write_index ++;
		
		if(packet_array_write_index == 5120)
			packet_array_write_index = 0;
	
		//info->dest_addr = pcie_dest_addr;
		//info->src_addr = pcie_src_addr;
		//info->size = data_size;
		
		schedule_work(&(info->tx_wq));
		
		//if(async)
		{
			//printk("%s[%d]\n",__FUNCTION__,__LINE__);
			kill_fasync(&async, SIGIO, POLL_IN);  //向打开设备文件的进程发出SIGIO信号  
		}
		i++;
	}
	while(0);

	//unmask
	__raw_writel(1,reg_vir + MSIX_IRQ(offset) + MSI_IRQ_ENABLE_SET);
	
	//printk("%s[%d]\n",__FUNCTION__,__LINE__);
	
	return IRQ_HANDLED;
}
#else
static irqreturn_t keystone_msi_handler(int irq, void *dev)
{
	return IRQ_HANDLED;
}
#endif

static irqreturn_t keystone_msi0_handler(int irq, void *dev)
{
	struct ks2_int_info *info = dev;
	int offset = irq - info->msi_irqs[0];
	printk("%s[%d]\n",__FUNCTION__,__LINE__);

	//mask	
	__raw_writel(1,reg_vir + MSIX_IRQ(offset) + MSI_IRQ_ENABLE_CLR);
	//printk("%s[%d]\n",__FUNCTION__,__LINE__);
	//ack
	__raw_writel(1,reg_vir + MSIX_IRQ(offset) + MSI0_IRQ_STATUS);
	__raw_writel(MSI_IRQ_OFFSET+offset, reg_vir + IRQ_EOI);
	
	__raw_writel(0xFFFFFFFF,reg_vir + MSIX_IRQ(offset)+MSI_IRQ_STATUS);

	//unmask
	__raw_writel(1,reg_vir + MSIX_IRQ(offset) + MSI_IRQ_ENABLE_SET);
	return IRQ_HANDLED;
}

/**
 * get_free_msi() - Get a free MSI number
 * @msi_irq_num - Maximum number of MSI irqs supported
 *
 * Checks for availability of MSI and returns the first available.
 */
static int get_free_msi(int msi_irq_num)
{
	int bit;

	do {
		bit = find_first_zero_bit(msi_irq_bits, msi_irq_num);

		if (bit >= msi_irq_num)
			return -ENOSPC;

	} while (test_and_set_bit(bit, msi_irq_bits));

	pr_debug(DRIVER_NAME ": MSI %d available\n", bit);

	return bit;
}

void arch_teardown_msi_irq(unsigned int irq)
{
	struct irq_data *irq_data = irq_get_chip_data(irq);
	struct keystone_pcie_info *info = irq_data_get_irq_chip_data(irq_data);

	if (info) {
		int pos = irq - irq_linear_revmap(info->msi_irqd, 0);

		irq_set_chip_and_handler(irq, NULL, NULL);
		irq_set_chip_data(irq, NULL);
		clear_bit(pos, msi_irq_bits);
		return;
	}
	pr_err(DRIVER_NAME
		": arch_teardown_msi_irq, can't find driver data\n");
}


/**
 * ti_k2h_set_inbound() - Setup inbound window
 * @in: address of struct ti_k2h_inb_window
 *
 * _NOTE_: in-bound region are fixed for BARs
 * in case of 32 bit: PCIE ADDRESS HIGHER 32 BITS ARE 0
 *
 * IB_REGION 0: BAR1
 * IB_REGION 1: BAR2
 * IB_REGION 2: BAR3
 * IB_REGION 3: BAR4
 *
 * in case of 64 bit: BAR0 BAR2 BAR4 will be asssigned to IB_REGIONS
 *
 * IB_REGION 0: BAR0-BAR1: --BAR0
 * IB_REGION 1: BAR2-BAR3: --BAR2
 * IB_REGION 2: BAR4-BAR5: --BAR4
 */

static int ti_k2h_set_inbound(struct ti_k2h_inb_window *in)
{
	u32 ib_region;
	//if (in->ib_start_hi == 0)/*32 bit mode*/
	//	ib_region = in->BAR_num-1;
	//else/*64 bit mode*/
		ib_region = in->BAR_num;
	__raw_writel(in->internal_addr, reg_vir + IB_OFFSET(ib_region));
	__raw_writel(in->ib_start_hi, reg_vir + IB_START_HI(ib_region));
	__raw_writel(in->ib_start_lo, reg_vir + IB_START_LO(ib_region));
	__raw_writel(in->BAR_num, reg_vir + IB_BAR(ib_region));
	return 0;
}


/**
 * ti_k2h_check_region()-check availability of continuous available regions.
 * @regions: no of continuous regions you are looking for.
 */

static u32  ti_k2h_check_regions(unsigned int  regions)
{
	int i = 0, j = 0;
	for (i = 0; i <= 31; i++) {
		unsigned  count = 0;
		if (status_out & 1 << i) {
			count++;
			if (count == regions)
				return i;
			for (j = i + 1; j <= 31; j++) {
				if (status_out & 1 << j) {
					count++;
					if (count == regions)
						return i;
				} else
					break;
				}
		}
	}
	pr_debug(DRIVER_NAME ": outbound regions not available");
	return XCEEDREG;
}


/**
 * ti_k2h_check_req_status()- this function check wether current request can
 * be served by increasing ob_size.
 * @out: pointer to struct ti_k2h_outb_region that contain request detail.
 * @ob_size: current ob_size.
 *
 * This function checks wether current request can be served along with older
 * mapping by increasing OB_SIZE.
 * Return values:
 * NMEM: if not able to serve.
 * INOB: if can be served by increasing OB_SIZE.
 */

static u32 ti_k2h_check_req_status(struct ti_k2h_outb_region *out, u32 ob_size)
{
	if (ob_size < 3) {
		struct list_head *tmp;
		u32 reg_current = 0;
		u32 reg_req = 0;
		list_for_each(tmp, &outb_req) {
			struct ti_k2h_outb_info *ptr =
						(struct ti_k2h_outb_info *)tmp;
				if ((ptr->size_req) % (8 * MB) != 0)
					reg_current +=
						(ptr->size_req) / (8 * MB)+1;
				else
					reg_current +=
						(ptr->size_req) / (8 * MB);
		}
		if ((out->size) % (8 * MB) != 0)
			reg_req += (out->size) / (8 * MB) + 1;
		else
			reg_req += (out->size) / (8 * MB);
		if ((reg_req + reg_current) <= 31)
			return INOB;
		else
			return NMEM;
	} else
		return NMEM;
}

#if 0
static int ti_k2h_set_msi_out_region(void)
{
	struct ti_k2h_msi_info msi;
	u32 offset;
	u32 mask, low;
	u32 ob_size = __raw_readl(reg_vir + OB_SIZE);
	u32 size_region;
	msi.msi_data = __raw_readl(reg_vir +
				LOCAL_CONFIG_OFFSET + MSI_DATA + MSI_OFF);
	msi.msi_addr_low = __raw_readl(reg_vir +
				LOCAL_CONFIG_OFFSET + MSI_LOW32 + MSI_OFF);
	msi.msi_addr_hi = __raw_readl(reg_vir +
				LOCAL_CONFIG_OFFSET + MSI_UP32 + MSI_OFF);
	
	printk("%s[%d]ob_size = %d\n",__FUNCTION__,__LINE__,ob_size);
	
	printk("%s[%d]msi_data = 0x%08x\n",__FUNCTION__,__LINE__,msi.msi_data);
	printk("%s[%d]msi_addr_low = 0x%08x\n",__FUNCTION__,__LINE__,msi.msi_addr_low);
	printk("%s[%d]msi_addr_low = 0x%08x\n",__FUNCTION__,__LINE__,msi.msi_addr_hi);
	
	if (ob_size == 3) {
		mask = 0xFF800000;
		size_region = 8 * MB;
	} else if (ob_size == 2) {
		mask = 0xFFC00000;
		size_region = 4 * MB;
	} else if (ob_size == 1) {
		mask = 0xFFE00000;
		size_region = 2 * MB;
	} else {
		mask = 0xFFF00000;
		size_region = 1 * MB;
	}

	/* fix region 31 for MSI generation*/

	if (1) {/* target is RC */

		/* XXX: Should be able to do this w/o outbound setting */

		low = msi.msi_addr_low & mask;
		offset = msi.msi_addr_low - low;

		printk("%s[%d]low = 0x%08x\n",__FUNCTION__,__LINE__,low);
		printk("%s[%d]offset = 0x%08x\n",__FUNCTION__,__LINE__,offset);
		__raw_writel(msi.msi_addr_hi,  reg_vir + OB_OFFSET_HI(0));
		__raw_writel(low | 0x1, reg_vir + OB_OFFSET_INDEX(0));
	}  
	return 0;
}
#endif

/**
 * ti_k2h_set_out_region()- serves request for outbound region mapping
 * to pcie address
 * @out: point to structure ti_k2h_outb_region passed by user space to
 * do a outbound mapping.
 */

static int ti_k2h_set_out_region(struct ti_k2h_outb_region *out)
{

	u32 regions;
	u32 start_region;
	struct ti_k2h_outb_info *info;
	struct list_head *tmp;
	u32 region_in_use = 0;
	u32 ob_size = __raw_readl(reg_vir + OB_SIZE);
	u32 region_size = (1 << ob_size) * MB;

	printk("%s[%d]ob_size = %d\n",__FUNCTION__,__LINE__,ob_size);

	list_for_each(tmp, &outb_req) {
		region_in_use += ((struct ti_k2h_outb_info *) tmp)->regions;
	}
	/*also keep in mind that some reasons may be disabled.*/

	if ((out->size) % (region_size) != 0)
		regions = ((out->size) / (region_size)) + 1;
	else
		regions = ((out->size) / (region_size));

	if (status_out == 0)
		return ti_k2h_check_req_status(out, ob_size);

	if (regions > 31)
		return ti_k2h_check_req_status(out, ob_size);

	start_region = ti_k2h_check_regions(regions);
	printk("%s[%d]start_region = %d\n",__FUNCTION__,__LINE__,start_region);


	if (start_region >= 0 && start_region <= 30) {
		u32 i, tr_size;
		u32 start_idx = out->ob_offset_idx;
		u32 hi_idx_add = out->ob_offset_hi;
		u32 max_lim = 0xFFFFFFFF;
		u32 carry = 0;
		tr_size = region_size;
		info = kmalloc(sizeof(struct ti_k2h_outb_info), GFP_KERNEL);
		if (info  == NULL)
			return FAIL;

		info->ob_offset_hi = out->ob_offset_hi;
		info->ob_offset_idx = out->ob_offset_idx;
		info->start_region = start_region;
		info->regions = regions;
		info->size_req = out->size;
		list_add_tail(&(info->list), &outb_req);

		for (i = 0; (i < regions) ; i++) //修改从region 1开始，由于PC 发起MSI中断时会修改Outbound region的HI_offset为1，原因尚不清楚
		{
			__raw_writel(start_idx | 0x1,
				reg_vir + OB_OFFSET_INDEX((start_region + i)));
			__raw_writel(hi_idx_add,
				reg_vir + OB_OFFSET_HI((start_region + i)));
			if ((max_lim - tr_size) < start_idx)
				carry = 1;
			start_idx += tr_size;
			hi_idx_add += carry;
			carry = 0;
			status_out = status_out & (~(1 << (start_region + i)));
		}

	} else
		return ti_k2h_check_req_status(out, ob_size);
	return 0;

}


/**
 * ti_k2h_clr_outb_map()- clear outbound mapping, specified by application
 * @out- point to structure passed by user space to free specified mapping.
 *
 * It is application's responsibility to clear all the mappings.
 *
 * TODO: Probably need to add locking for the list/status manipulation
 */

static int ti_k2h_clr_outb_map(struct ti_k2h_outb_region *out)
{
	struct list_head *pos, *q;
	struct ti_k2h_outb_info *temp;
	u32 i;
	u32 start;
	list_for_each_safe(pos, q, &outb_req) {
		 struct ti_k2h_outb_info *ptr = (struct ti_k2h_outb_info *)pos;
		 if ((ptr->ob_offset_hi == out->ob_offset_hi) &&
				(ptr->ob_offset_idx == out->ob_offset_idx)) {

			temp = list_entry(pos, struct ti_k2h_outb_info, list);
			start = ptr->start_region;
			for (i = 0; (i < ptr->regions) ; i++) {
				__raw_writel(0,
					reg_vir + OB_OFFSET_INDEX(start + i));
				__raw_writel(0,
					reg_vir + OB_OFFSET_HI(start + i));
				status_out = status_out | ((1 << (start + i)));
			}
			list_del(pos);
			kfree(temp);
		}
	}
	return 0;
}

/**
 * free_outb_map()-- iterate over the list of successfull outbound requests
 * and free them.
 */

static int ti_k2h_free_outb_map(void)
{
	struct list_head *pos, *q;
	struct ti_k2h_outb_info *temp;
	list_for_each_safe(pos, q, &outb_req) {
		temp = list_entry(pos, struct ti_k2h_outb_info, list);
		list_del(pos);
		kfree(temp);
	}
	return 0;
}


/**
 * ti_k2h_pciess_register_access() - access pciess registers
 * @arg: Userspace pointer holding struct ti_k2h_pciess_regs data.
 * 		IN	: in case mode is write register
 * 		INOUT	: in case mode is read register
 */

static int ti_k2h_pciess_register_access(void __user *arg)
{
	struct ti_k2h_pciess_regs regs;

	if (copy_from_user(&regs, arg, sizeof(regs)))
		return -EFAULT;

	if (regs.mode == SET_REGS)
		__raw_writel(regs.value, reg_vir + regs.offset);
	else
	{
		regs.value = __raw_readl(reg_vir + regs.offset);
		if(copy_to_user(arg, &regs, sizeof(regs)))
			return -EFAULT;
	}
	return 0;
}

/**
 * ti_k2h_pciess_Bar0_access() - access pciess registers
 * @arg: Userspace pointer holding struct ti_k2h_pciess_regs data.
 * 		IN	: in case mode is write register
 * 		INOUT	: in case mode is read register
 */

static int ti_k2h_pciess_Bar1_access(void __user *arg)
{
	struct ti_k2h_pciess_regs regs;

	if (copy_from_user(&regs, arg, sizeof(regs)))
		return -EFAULT;

	if (regs.mode == SET_REGS)
	{
		printk("%s[%d]regs.value[0x%x]\n",__FUNCTION__,__LINE__,regs.value);
		__raw_writel(regs.value, ks2reg_vir + regs.offset);
	}
	else
	{
		regs.value = __raw_readl(ks2reg_vir + regs.offset);
		if(copy_to_user(arg, &regs, sizeof(regs)))
			return -EFAULT;
		
		printk("%s[%d]regs.value[0x%x]\n",__FUNCTION__,__LINE__,regs.value);
	}
	return 0;
}





/**
 * query_bar_info() - to get bar size and address from user space
 * @arg: Userspace pointer to struct ti_k2h_bar_info structure
 *
 * This is INOUT parameter.
 * 	IN	: BAR number
 * 	OUT	: BAR info
 */

static int query_bar_info(void __user *arg)
{
	u32 addr;
	u32 offset = 4;
	struct ti_k2h_bar_info bar_info;

	if (copy_from_user(&bar_info, arg, sizeof(bar_info)))
		return -EFAULT;

	addr = __raw_readl(reg_vir + LOCAL_CONFIG_OFFSET +
					BAR0 + (bar_info.bar_num) * offset);
	bar_info.bar_addr = addr & 0XFFFFFFF0;
	__raw_writel(0xFFFFFFFF, reg_vir + LOCAL_CONFIG_OFFSET +
					BAR0 + (bar_info.bar_num) * offset);
	bar_info.bar_size = __raw_readl(reg_vir + LOCAL_CONFIG_OFFSET +
					BAR0 + (bar_info.bar_num) * offset);
	bar_info.bar_size = ~(bar_info.bar_size & 0xFFFFFFF0) + 1;
	__raw_writel(addr, reg_vir + LOCAL_CONFIG_OFFSET +
					BAR0 + (bar_info.bar_num) * offset);

	if(copy_to_user(arg, &bar_info, sizeof(bar_info)))
		return -EFAULT;

	return 0;
}



static int get_bar_info(struct ti_k2h_bar_info *bar_info)
{
	u32 addr;
	u32 offset = 4;
	//32 bit or 64 bit
	addr = __raw_readl(reg_vir + LOCAL_CONFIG_OFFSET +
					BAR0 + (bar_info->bar_num) * offset);
	bar_info->bar_addr = addr & 0XFFFFFFF0;
	__raw_writel(0xFFFFFFFF, reg_vir + LOCAL_CONFIG_OFFSET +
					BAR0 + (bar_info->bar_num) * offset);
	bar_info->bar_size = __raw_readl(reg_vir + LOCAL_CONFIG_OFFSET +
					BAR0 + (bar_info->bar_num) * offset);
	bar_info->bar_size = ~(bar_info->bar_size & 0xFFFFFFF0) + 1;
	__raw_writel(addr, reg_vir + LOCAL_CONFIG_OFFSET +
					BAR0 + (bar_info->bar_num) * offset);

	return 0;
}



/**
 * ti816x_pcie_fault() - ARM abort handler for PCIe nonposted completion aborts
 * @addr: Address target on which the fault generated
 * @fsr: CP15 fault status register value
 * @regs: Pointer to register structure on abort
 *
 * Handles precise abort caused due to PCIe operation.
 *
 * Note that we are relying on virtual address filtering to determine if the
 * target of the precise aborts was a PCIe module access (i.e., config, I/O,
 * register) and only handle such aborts. We could check PCIe error status to
 * confirm if the abort was caused due to non-posted completion status received
 * by PCIESS, but this may not always be true and aborts from some downstream
 * devices, such as PCI-PCI bridges etc may not result into error status bit
 * getting set.
 *
 * Ignores and returns abort as unhandled otherwise.
 *
 * Also note that, using error status check (as was done in earlier
 * implementation) would also handle failed memory accesses (non-posted), but
 * address filerting based handling will cause aborts for memory accesses as the
 * addresses will be outside the PCIESS module space. This seems OK, as any
 * memory access after enumeration is sole responsibility of the driver and the
 * system integrator (e.g., access failures due to hotplug, suspend etc). If
 * using error check based handling, we also need to clear PCIe error status on
 * detecting errors.
 *
 * Note: Due to specific h/w implementation, we can't be sure of what kind of
 * error occurred (UR Completion, CA etc) and all we get is raw error IRQ status
 * and probably SERR which indicate 'some kind of' error - fatal or non-fatal is
 * received/happened.
 */
#if 0
static int ti816x_pcie_fault(unsigned long addr, unsigned int fsr,
						struct pt_regs *regs)
{
	unsigned long instr = *(unsigned long *)regs->ARM_pc;

	pr_debug(DRIVER_NAME ": Data abort: address = 0x%08lx "
				"fsr = 0x%03x PC = 0x%08lx LR = 0x%08lx",
				addr, fsr, regs->ARM_pc, regs->ARM_lr);

	/* Note: Only handle PCIESS module space access */
	if ((addr < reg_vir) || (addr >= (reg_vir + SZ_16K)))
		return -1;

	/*
	* Mimic aborted read of all 1's as required to detect device/function
	* absence - check if the instruction that caused abort was a LOAD,
	*/
	if ((instr & 0x0c100000) == 0x04100000) {
		int reg = (instr >> 12) & 15;
		unsigned long val;

		if (instr & 0x00400000)
			val = 255;
		else
			val = -1;

		regs->uregs[reg] = val;
	}

	regs->ARM_pc += 4;

	pr_debug(DRIVER_NAME ": Handled PCIe abort\n");
	return 0;
}

#endif

/**
 * ti_k2h_send_msi()-- send msi interrupt to RC/EP
 * @bar: it points to structure containing bar0 and bar1 address of
 * targeted ep.
 *
 * it configures PCIESS registers so that it can send MSI to any RC or EP.
 * in case of 64 bit mode application must set bar0 and bar1 both while in
 * case of 32 bit mode application must set bar1 0 always.
 *
 * There is a register at an offset 0x54 from ox51000000 on every RC and EP
 * that must be written from remote EP / RC to genrate an interrupt.
 *
 *
 * TODO: 64 bit MSI.
 * Only 32 bit MSI is enabled. so as for now MSI_UP32 addr is always zero.
 */

#if 0
static int ti_k2h_send_msi(struct ti_k2h_msi_bar *bar)
{
	struct ti_k2h_msi_info msi;
	u32 offset;
	u32 mask, low;
	u32 ob_size = __raw_readl(reg_vir + OB_SIZE);
	u32 size_region;
	msi.msi_data = __raw_readl(reg_vir +
				LOCAL_CONFIG_OFFSET + MSI_DATA + MSI_OFF);
	msi.msi_addr_low = __raw_readl(reg_vir +
				LOCAL_CONFIG_OFFSET + MSI_LOW32 + MSI_OFF);
	msi.msi_addr_hi = __raw_readl(reg_vir +
				LOCAL_CONFIG_OFFSET + MSI_UP32 + MSI_OFF);
	
	printk("%s[%d]ob_size = %d\n",__FUNCTION__,__LINE__,ob_size);
	
	printk("%s[%d]msi_data = 0x%08x\n",__FUNCTION__,__LINE__,msi.msi_data);
	printk("%s[%d]msi_addr_low = 0x%08x\n",__FUNCTION__,__LINE__,msi.msi_addr_low);
	printk("%s[%d]msi_addr_low = 0x%08x\n",__FUNCTION__,__LINE__,msi.msi_addr_hi);
	
	if (ob_size == 3) {
		mask = 0xFF800000;
		size_region = 8 * MB;
	} else if (ob_size == 2) {
		mask = 0xFFC00000;
		size_region = 4 * MB;
	} else if (ob_size == 1) {
		mask = 0xFFE00000;
		size_region = 2 * MB;
	} else {
		mask = 0xFFF00000;
		size_region = 1 * MB;
	}
	
	//msi.msi_addr_low = 0xbf000000;

	/* fix region 31 for MSI generation*/

	if (1) {/* target is RC */

		/* XXX: Should be able to do this w/o outbound setting */

		low = msi.msi_addr_low & mask;
		offset = msi.msi_addr_low - low;

		printk("%s[%d]low = 0x%08x\n",__FUNCTION__,__LINE__,low);
		printk("%s[%d]offset = 0x%08x\n",__FUNCTION__,__LINE__,offset);
		__raw_writel(msi.msi_addr_hi,  reg_vir + OB_OFFSET_HI(31));
		__raw_writel(low | 0x1, reg_vir + OB_OFFSET_INDEX(31));
		
		printk("%s[%d]reg_mem = 0x%08x\n",__FUNCTION__,__LINE__,reg_mem);
		
		__raw_writel(msi.msi_data, reg_mem + 0 + 0 * size_region);
		__raw_writel(msi.msi_data, reg_mem + offset + 0 * size_region);
	}  

	/*
	 * FIXME: Need to restore outbound window
	 */

	return 0;
}



static int ti_k2h_send_msi_1(struct ti_k2h_msi_bar *bar)
{
	struct ti_k2h_msi_info msi;
	u32 offset;
	u32 mask, low;
	u32 ob_size = __raw_readl(reg_vir + OB_SIZE);
	u32 size_region;
	msi.msi_data = __raw_readl(reg_vir +
				LOCAL_CONFIG_OFFSET + MSI_DATA + MSI_OFF);
	msi.msi_addr_low = __raw_readl(reg_vir +
				LOCAL_CONFIG_OFFSET + MSI_LOW32 + MSI_OFF);
	msi.msi_addr_hi = __raw_readl(reg_vir +
				LOCAL_CONFIG_OFFSET + MSI_UP32 + MSI_OFF);
	
	printk("%s[%d]ob_size = %d\n",__FUNCTION__,__LINE__,ob_size);
	
	printk("%s[%d]msi_data = 0x%08x\n",__FUNCTION__,__LINE__,msi.msi_data);
	printk("%s[%d]msi_addr_low = 0x%08x\n",__FUNCTION__,__LINE__,msi.msi_addr_low);
	printk("%s[%d]msi_addr_low = 0x%08x\n",__FUNCTION__,__LINE__,msi.msi_addr_hi);
	
	if (ob_size == 3) {
		mask = 0xFF800000;
		size_region = 8 * MB;
	} else if (ob_size == 2) {
		mask = 0xFFC00000;
		size_region = 4 * MB;
	} else if (ob_size == 1) {
		mask = 0xFFE00000;
		size_region = 2 * MB;
	} else {
		mask = 0xFFF00000;
		size_region = 1 * MB;
	}

	/* fix region 31 for MSI generation*/

	if (bar == NULL) {/* target is RC */

		/* XXX: Should be able to do this w/o outbound setting */

		low = msi.msi_addr_low & mask;
		offset = msi.msi_addr_low - low;

		printk("%s[%d]low = 0x%08x\n",__FUNCTION__,__LINE__,low);
		printk("%s[%d]offset = 0x%08x\n",__FUNCTION__,__LINE__,offset);
		__raw_writel(msi.msi_addr_hi,  reg_vir + OB_OFFSET_HI(31));
		__raw_writel(low | 0x1, reg_vir + OB_OFFSET_INDEX(31));
		__raw_writel(msi.msi_data, reg_mem + offset + 31 * size_region);

	} else {
		bar->bar0 += 0x54;
		low = bar->bar0 & mask;
		offset = bar->bar0 - low;

		printk("%s[%d]low = 0x%08x\n",__FUNCTION__,__LINE__,low);
		printk("%s[%d]offset = 0x%08x\n",__FUNCTION__,__LINE__,offset);
		
		__raw_writel(bar->bar1, reg_vir + OB_OFFSET_HI(31));
		__raw_writel(low | 0x1, reg_vir + OB_OFFSET_INDEX(31));
		__raw_writel(msi.msi_data, reg_mem + offset + 31 * size_region);

	}

	/*
	 * FIXME: Need to restore outbound window
	 */

	return 0;
}
#endif

static int ti_k2h_ep_pcie_open(struct inode *inode, struct file *file)
{
	pr_debug(DRIVER_NAME ": file opened successfull");
	status_out = 0x7ffffff0;
	atomic_set(&counter, 0);
	return 0;
}


static int ti_k2h_send_intx(void __user *arg)
{
	__raw_writel(1, reg_vir + LEGACY_N_IRQ_ENABLE_SET(0));
	__raw_writel(1, reg_vir + EP_IRQ_SET);

	return 0;
}







/**
 * ti_k2h_pcie_ioctl() - Application interface for application
 *
 * Provides interface to the application code to initialize management area.
 * There are interfaces for setting in-bound and out-bound window as and when
 * needed. Application can query start address of dedecated physical area and
 * management area info. Application can also access/update the status of
 * different registers in PCIESS. There is an interface for MSI genertaion too.
 *
 * TODO: Move copy_*s inside respective handlers.
 */

static long ti_k2h_ep_pcie_ioctl(struct file *file, unsigned int cmd,
							unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	unsigned int size;
	unsigned int status;

	switch (cmd) {
	case TI_K2H_SET_INBOUND:
	{
		struct ti_k2h_inb_window inb;
		if (copy_from_user(&inb, argp, sizeof(inb)))
			return -EFAULT;
		ti_k2h_set_inbound(&inb);
	}
	break;

	case TI_K2H_SET_OUTBOUND_SIZE:
	{
		printk("%s[%d]arg = %d\n",__FUNCTION__,__LINE__,arg);
		if (get_user(size, (unsigned int *)arg))
		//if (copy_from_user(&size, argp, sizeof(unsigned int)))
			return -EFAULT;
		printk("%s[%d]size = %d\n",__FUNCTION__,__LINE__,size);
		__raw_writel(size, reg_vir + OB_SIZE);
	}
	break;

	case TI_K2H_SET_OUTBOUND:
	{
		struct ti_k2h_outb_region outb;
		if (copy_from_user(&outb, argp, sizeof(outb)))
			return -EFAULT;
		return ti_k2h_set_out_region(&outb);
	}
	break;

	case TI_K2H_ACCESS_REGS:
		ti_k2h_pciess_register_access(argp);		
	break;

	case TI_K2H_SEND_MSI:
	{
		#if 0
		struct ti_k2h_msi_bar msi;
		printk("%s[%d]\n",__FUNCTION__,__LINE__);
		//if (copy_from_user(&msi, argp, sizeof(msi)))
		//	return -EFAULT;
		ti_k2h_send_msi(&msi);
		#endif
	}
	break;

	case TI_K2H_GET_PCIE_MEM_INFO:
	{
		struct ti_k2h_pcie_mem_info mem;
		mem.base = ti_k2h_ep_mem_start;
		mem.size = ti_k2h_ep_mem_size;
		if (copy_to_user(argp, &mem, sizeof(mem)))
			return -EFAULT;
	}
	break;

	case TI_K2H_CUR_TIME:
		printk("TI_K2H_CUR_TIME IN\n");
		if (put_user(jiffies, (unsigned int *)arg))
			return -EFAULT;
	break;

	case TI_K2H_GET_OUTBOUND_STATUS:
		if (put_user(status_out, (unsigned int *)arg))
			return -EFAULT;
	break;
	
	case TI_K2H_SET_OUTBOUND_STATUS:
		if (get_user(status, (unsigned int *)arg))
			return -EFAULT;
		printk("TI_K2H_SET_OUTBOUND_STATUS:status[0x%08x]\n",status);
		status_out = status;
	break;

	case TI_K2H_CLR_OUTBOUND_MAP:
	{
		struct ti_k2h_outb_region outb;
		if (copy_from_user(&outb, argp, sizeof(outb)))
			return -EFAULT;
		ti_k2h_clr_outb_map(&outb);
	}
	break;

	case TI_K2H_GET_INTR_CNTR:
	{
		int temp = atomic_read(&counter);
		if (put_user(temp, (int *)arg))
			return -EFAULT;
	}
	break;

	case TI_K2H_GET_BAR_INFO:
		query_bar_info(argp);
	break;

	case TI_K2H_SEND_INTX:
	{
		printk("%s[%d]\n",__FUNCTION__,__LINE__);
		//if (copy_from_user(&msi, argp, sizeof(msi)))
		//	return -EFAULT;
		ti_k2h_send_intx(argp);
	}
	break;


	/*20151225 add by langsen*/
	case TI_K2H_TRIGGER_IPCGRX:
	{


	}
	break;


	case TI_K2H_ACCESS_MEM_REG:
	{
		ti_k2h_pciess_Bar1_access(argp);
	}
	break;

	default:
		pr_err("Invalid cmd passed\n");
	}

	return 0;
}

/**
 * ti_k2h_pcie_poll()-- this function supports poll call from user space.
 *
 * on receving interrupt from other EP/RC, it wake up and
 * check data availabilty, if data is available for reading
 * it return POLLIN otherwise it return 0.
 *
 */

static unsigned int ti_k2h_pcie_poll(struct file *filp, poll_table *wait)
{
	unsigned int mask = 0;
	
	//down(&sem_poll);
	//poll_wait(filp, &readq, wait);
	//if (atomic_read(&irq_raised) == 1)
	//	mask |= POLLIN;    /* readable */
	//atomic_set(&irq_raised, 0);
	//up(&sem_poll);
	
	return mask;
}


/**
 * ti_k2h_ep_pcie_mmap() - Provide userspace mapping for specified kernelmemory
 *
 * @filp: File private data
 * @vma: User virtual memory area to map to
 *
 * At present, we are mapping entire physical contiguous dedicated area for EP
 * and pci space(0x20000000-0x2fffffff).
 * application has to know start address of dedicated area, it can query about
 * it, using specified ioctl.
 */

static int ti_k2h_ep_pcie_mmap(struct file *filp, struct vm_area_struct *vma)
{
	int ret = -EINVAL;
	unsigned long sz = vma->vm_end - vma->vm_start;
	unsigned int addr = (unsigned int)vma->vm_pgoff << PAGE_SHIFT;
	printk("Mapping %#lx bytes from address %#x\n"
			, sz, addr);
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	ret = remap_pfn_range(vma, vma->vm_start,
					vma->vm_pgoff,
						sz, vma->vm_page_prot);
	return ret;
}

/**
 * ti_k2h_ep_pcie_close()-- free all the successfull outbound request and set regios
 * status to default.
 */

static int ti_k2h_ep_pcie_close(struct inode *inode, struct file *filp)
{
	/* free all list of outbound mapping*/
    fasync_helper(-1, filp, 0, &async); //注册上层调用进程的信息，上层调用fcntl设置FASYNC会调用这个系统调用  
	return 0;
}


/**
 * ti_k2h_setup_msi() - it sets up msi related configuration.
 *
 * _NOTE_:  disable legacy interrupt and enable MSi capability.
 *
 * __TO_DO__:
 */

static int ti_k2h_setup_msi()
{
	int i = 0;
	
//	__raw_writel(0x0, reg_vir + 0x18c);
//	__raw_writel(__raw_readl(reg_vir + STATUS_COMMAND +
//			LOCAL_CONFIG_OFFSET) | (1 << 10),
//			reg_vir + STATUS_COMMAND + LOCAL_CONFIG_OFFSET);
//	__raw_writel(__raw_readl(reg_vir + LOCAL_CONFIG_OFFSET +
//			MSI_CAP + MSI_OFF) | (1 << 16),
//			reg_vir + LOCAL_CONFIG_OFFSET + MSI_CAP + MSI_OFF);
	
	for(i=0;i<8;i++)
	{
		__raw_writel(0xFFFFFFFF, reg_vir + MSI0_IRQ_ENABLE_SET+(i<<4));
	}
	printk(":msi capability setup done\n");
	return 0;
}


/**
 * ti_k2h_ep_pcie_handle()- interrupt handler
 *
 * notify about data in buffers, and signal select/poll
 * implementation about data availability.
 */
static irqreturn_t ti_k2h_ep_pcie_handle(int irq, void *dev)
{
	int i = 0;
	printk("Enter ti_k2h_ep_pcie_handle:irq[%d]\n",irq);
	//__raw_writel(1, reg_vir + LEGACY_N_IRQ_STATUS_RAW(0));
	
	//while (i <= 1500) {
	//	if (__raw_readl(reg_vir + MSI0_IRQ_STATUS) != 0) {
	//		if (device_id == 0xb801)
	//			__raw_writel(0xFFFFFFFF,
	//				reg_vir + MSI0_IRQ_STATUS);
	//		if (device_id == 0xb800)
	//			__raw_writel(0x0,
	//				reg_vir + MSI0_IRQ_STATUS);
	//		__raw_writel(1, reg_vir + IRQ_EOI);
	//		atomic_set(&irq_raised, 1);
	//		atomic_inc(&counter);
	//		wake_up_interruptible(&readq);
	//	}
	//	i++;
	//}
	printk("%s[%d]\n",__FUNCTION__,__LINE__);
	return IRQ_HANDLED;
}

static irqreturn_t pcie_err_irq_handler(int irq, void *reg_virt)
{
	int ret = IRQ_NONE, status;

	status = __raw_readl(reg_virt + ERR_IRQ_STATUS_RAW) & ERR_FATAL_IRQ;
	if (status) {
		/* The PCIESS interrupt status buts are "write 1 to clear" */
		__raw_writel(status, reg_virt + ERR_IRQ_STATUS);
		pr_err(DRIVER_NAME ": PCIE Fatal error detected\n");
		ret = IRQ_HANDLED;
	}
	return ret;
}

static int get_and_clear_err(void __iomem *reg_virt)
{
	int status = __raw_readl(reg_virt + ERR_IRQ_STATUS_RAW);

	if (status) {
		/* The PCIESS interrupt status buts are "write 1 to clear" */
		__raw_writel(status, reg_virt + ERR_IRQ_STATUS);

		/*
		 * Clear all errors. We are not worried here about individual
		 * status as no specific handling is required.
		 */
		__raw_writew(0xffff, reg_virt + SPACE0_LOCAL_CFG_OFFSET +
				PCI_STATUS);
	}
	return status;
}

static int ti_k2h_ep_pcie_fasync(int fd, struct file *filp, int mode)  
{  
    printk("application  fasync!\n");  
    return fasync_helper(fd, filp, mode, &async); //注册上层调用进程的信息，上层调用fcntl设置FASYNC会调用这个系统调用  
}  


/**
 * ti_k2h_pci_fops - Declares supported file access functions
 */
static const struct file_operations ti_k2h_ep_pcie_fops = {
	.owner          = THIS_MODULE,
	.mmap           = ti_k2h_ep_pcie_mmap,
	.open           = ti_k2h_ep_pcie_open,
	.release        = ti_k2h_ep_pcie_close,
	.poll           = ti_k2h_pcie_poll,
	.unlocked_ioctl = ti_k2h_ep_pcie_ioctl,
	.fasync 		= ti_k2h_ep_pcie_fasync, 
};

/* keystone pcie device tree match tables */
static const struct of_device_id keystone_pci_match_ids[] __initconst = {
	{
		.type = "pci",
		.compatible = "ti,keystone2-pci",
	},
	{}
};

static int __init keystone_pcie_get_resources(struct device_node *node,
					struct ks2_int_info *info)
{
	int i = 0;

	/* support upto 4 legacy irqs */
	do {
		if(i<=3)
		{
			info->legacy_irqs[i] =
				irq_of_parse_and_map(node, i);
			printk("info->legacy_irqs[%d]=%d\n",i,info->legacy_irqs[i]);
			
			if (info->legacy_irqs[i] < 0)
				break;
		}else if((i>=4)&&(i<=11))
		{
			info->msi_irqs[i-4] =
				irq_of_parse_and_map(node, i);
			
			printk("info->msi_irqs[%d]=%d\n",i-4,info->msi_irqs[i-4]);
			if (info->msi_irqs[i-4] < 0)
				break;
		}
		else if(i==12)
		{
			info->error_irq =
				irq_of_parse_and_map(node, i);
			printk("info->error_irq=%d\n",info->error_irq);
			
			if (info->error_irq < 0)
				break;
		}
		i++;
	} while (i < 13);


	return 0;
}

static int __init keystone_pcie_interrupt_set(struct device_node *np,
						int domain)
{
	struct keystone_edma_info *edma_info;
	const struct of_device_id *of_id;
	int err = -EINVAL, i,j;
	struct clk *pcie_clk;
	int port = 0;
	char			irq_name[30];
	
	printk("%s[%d]\n",__FUNCTION__,__LINE__);

	//edma_info = kzalloc(sizeof(*edma_info), GFP_KERNEL);
	//if (!edma_info) {
	//	printk("%s[%d]\n",__FUNCTION__,__LINE__);
	//	return -ENOMEM;
	//}

	of_id = of_match_node(keystone_pci_match_ids, np);


	err = keystone_pcie_get_resources(np, &pcie_int_info);
	if (err < 0) {
		printk("%s[%d]\n",__FUNCTION__,__LINE__);
		goto err1;
	}

	for(i=0;i<13;i++)
	{
		//if((i>=0)&&(i<=3))
		//{
		//	sprintf(irq_name, "pcie-legacy-irq%d", i);
		//	if (request_irq(pcie_int_info.legacy_irqs[i], keystone_legacy_irq_handler_1, IRQF_SHARED,
		//			irq_name, &ti_k2h_ep_pcie_cdev) < 0) {
		//		printk("%s[%d]\n",__FUNCTION__,__LINE__);
		//		goto err;
		//	}
		//}
		//else 

		/*if(i == 4){
			sprintf(irq_name, "pcie-msi-irqs%d", i-4);
			if (request_irq(pcie_int_info.msi_irqs[i-4], keystone_msi0_handler, IRQF_SHARED,
					irq_name, &pcie_int_info) < 0) {
				printk("%s[%d]\n",__FUNCTION__,__LINE__);
				goto err;
			}
			else
			{
				printk("request msi interrupt success\n");
			}
		}
		else if((i >= 5)&&(i <= 11))
		*/
		if((i >= 4)&&(i <= 11))
		{
			sprintf(irq_name, "pcie-msi-irqs%d", i-4);
			if (request_irq(pcie_int_info.msi_irqs[i-4], keystone_msi_handler, IRQF_SHARED,
					irq_name, &pcie_int_info) < 0) {
				printk("%s[%d]\n",__FUNCTION__,__LINE__);
				goto err;
			}
			else
			{
				printk("request msi interrupt success\n");
			}
		}
		else if(i == 12)
		{
			if (request_irq(pcie_int_info.error_irq, pcie_err_irq_handler, IRQF_SHARED,
					"pcie-error-irq", reg_vir) < 0) {
				printk("%s[%d]\n",__FUNCTION__,__LINE__);
				goto err;
			}
		}

		
	}
	
	return 0;
err1:
err:
	//kfree(edma_info);
	return err;
}

static int __init ti_k2h_pcie_interrupt_set(void)
{
	struct device_node *np = NULL;
	int ret = 0, domain = 0;


	for_each_matching_node(np, keystone_pci_match_ids) {
	printk("%s[%d]\n",__FUNCTION__,__LINE__);
		if (of_device_is_available(np))
		{
			printk("%s[%d]\n",__FUNCTION__,__LINE__);
			ret = keystone_pcie_interrupt_set(np, domain);
		}
		domain++;
	}
	return ret;
}



static int __init keystone_pcie_get_resources_all(struct device_node *node,
					struct keystone_pcie_info *info)
{
	unsigned long long pci_addr, cpu_addr, size;
	int rlen, pna = of_n_addr_cells(node), err;
	int np = pna + 5, memno = 0, iono = 0;
	struct resource *res;
	u32 pci_space, temp;
	const u32 *ranges;
	int i = 0;

	err = of_address_to_resource(node, 0, &info->cfg_resource);
	if (err < 0) {
		pr_err(DRIVER_NAME ": Not found reg property\n");
		return -EINVAL;
	}

	info->reg_cfg_virt = ioremap_nocache(info->cfg_resource.start,
					 resource_size(&info->cfg_resource));
	if (info->reg_cfg_virt == 0) {
		pr_err(DRIVER_NAME ": Couldn't map reg cfg address\n");
		return -ENOMEM;
	}

	ranges = of_get_property(node, "ranges", &rlen);
	if (ranges == NULL) {
		pr_err(DRIVER_NAME ": no range property\n");
		err = -EINVAL;
		goto err;
	}

	printk("rlen[%d]np[%d]pna[%d]\n",rlen,np,pna);
	
	/* Parse the ranges */
	while ((rlen -= np * 4) >= 0) {
		for(i = 0;i<np;i++)
		{
			printk("ranges[%d][0x%08x]\n",i,of_read_number(ranges+i, 1));
		}
		/* Read next ranges element */
		pci_space = of_read_number(&ranges[0], 1);
		pci_addr = of_read_number(ranges + 1, 2);
		cpu_addr = of_translate_address(node, ranges + 3);
		size = of_read_number(ranges + pna + 3, 2);
		ranges += np;


		printk("pci_space[0x%08x]\n",pci_space);
		printk("pci_addr[0x%08x]\n",pci_addr);
		printk("cpu_addr[0x%08x]\n",cpu_addr);
		printk("size[0x%08x]\n",size);


		if (cpu_addr == OF_BAD_ADDR || size == 0)
			continue;

		/* Act based on address space type */
		res = NULL;
		switch ((pci_space >> 24) & 0x3) {
		case 1:         /* PCI IO space */
			pr_info(DRIVER_NAME
				": IO 0x%016llx..0x%016llx -> 0x%016llx\n",
				cpu_addr, cpu_addr + size - 1, pci_addr);

			/* We support only one IO range */
			if (iono >= 1) {
				pr_info(DRIVER_NAME
					":--> Skipped (too many) IO res!\n");
				continue;
			}
			res = &info->io_resource;
			res->flags = IORESOURCE_IO;
			res->start = pci_addr;
			res->name = "PCI I/O";
			iono++;
			break;
		case 2:         /* PCI Memory space */
		case 3:         /* PCI 64 bits Memory space */
			pr_info(DRIVER_NAME
				": MEM 0x%016llx..0x%016llx -> 0x%016llx %s\n",
				cpu_addr, cpu_addr + size - 1, pci_addr,
				(pci_space & 0x40000000) ? "Prefetch" : "");

			/* We support only 2 memory ranges */
			if (memno >= 1) {
				pr_info(DRIVER_NAME ":--> Skipped (too many)!\n");
				continue;
			}
			res = &info->mem_resource;
			res->flags = IORESOURCE_MEM;
			if (pci_space & 0x40000000) {
				pr_info(DRIVER_NAME
				": Skipped, don't support prefetch memory!\n");
				continue;
			}

			res->start = cpu_addr;
			res->name = "PCI Memory";
			memno++;
			break;
		default:
			pr_warn(DRIVER_NAME ": Unknown resource\n");
			break;
		}
		info->num_mem_resources = memno;

		if (res != NULL) {
			res->end = res->start + size - 1;
			res->parent = NULL;
			res->sibling = NULL;
			res->child = NULL;
		}
	}

	if ((memno == 0) || iono == 0) {
		pr_err(DRIVER_NAME ": No Mem/IO resources defined for PCIE\n");
		err = -EINVAL;
		goto err;
	}

	/* Get dma range for inbound memory access */
	res = &info->inbound_dma_res;
	res->start = 0;
	/* maximum 1G space */
	size = MAX_DMA_RANGE;
	res->end = size - 1;
	res->flags = IORESOURCE_MEM | IORESOURCE_PREFETCH;

	/* Get dma-ranges property */
	ranges = of_get_property(node, "dma-ranges", &rlen);
	if (ranges == NULL) {
		pr_err(DRIVER_NAME ": no dma range property\n");
		err = -EINVAL;
		goto err;
	}

	/* read ranges element */
	pci_space = of_read_number(&ranges[0], 1);
	pci_addr = of_read_number(ranges + 1, 2);
	cpu_addr = of_translate_address(node, ranges + 3);
	size = of_read_number(ranges + pna + 3, 2);
	if (cpu_addr == OF_BAD_ADDR || size == 0) {
		pr_err(DRIVER_NAME ": Invalid cpu address in dma range\n");
		err = -EINVAL;
		goto err;
	}

	temp = (pci_space >> 24) & 3;
	/* We only care about memory */
	if (temp != 2 && temp != 3) {
		pr_err(DRIVER_NAME ": Invalid memory in dma range\n");
		err = -EINVAL;
		goto err;
	}

	if (size > MAX_DMA_RANGE) {
		pr_err(DRIVER_NAME ": Invalid dma range size\n");
		err = -EINVAL;
		goto err;
	}

	if (!(pci_space & 0x40000000))
		res->flags &= ~IORESOURCE_PREFETCH;

	res->start = pci_addr;
	res->end = res->start + size - 1;

	/* support upto 4 legacy irqs */
	do {
		info->legacy_irqs[info->num_legacy_irqs] =
			irq_of_parse_and_map(node, info->num_legacy_irqs);
		if (info->legacy_irqs[info->num_legacy_irqs] < 0)
			break;
		info->num_legacy_irqs++;
	} while (info->num_legacy_irqs < MAX_LEGACY_HOST_IRQS);

	if (!info->num_legacy_irqs) {
		pr_err(DRIVER_NAME ": No host legacy irqs defined\n");
		err = -EINVAL;
		goto err;
	}
	pr_info(DRIVER_NAME
		": pcie - number of legacy irqs = %d\n", info->num_legacy_irqs);

#ifdef CONFIG_PCI_MSI
	/*
	 * support upto 32 MSI irqs. Actual numbers configured through
	 * dt from index 4 to 11
	 */
	do {
		info->msi_host_irqs[info->num_msi_host_irqs] =
			irq_of_parse_and_map(node, info->num_legacy_irqs +
						info->num_msi_host_irqs);
		if (info->msi_host_irqs[info->num_msi_host_irqs] < 0)
			break;
		info->num_msi_host_irqs++;
	} while (info->num_msi_host_irqs < MAX_MSI_HOST_IRQS);

	if (!info->num_msi_host_irqs) {
		pr_err(DRIVER_NAME ": No MSI host irqs defined\n");
		err = -EINVAL;
		goto err;
	}

	info->num_msi_irqs = info->num_msi_host_irqs * 4;
	if (info->num_msi_irqs > MAX_MSI_IRQS) {
		info->num_msi_irqs = MAX_MSI_IRQS;
		pr_warn(DRIVER_NAME
			": MSI irqs exceeds maximum capacity. Set to max.\n");
	}

	pr_info(DRIVER_NAME
		": pcie - number of MSI host irqs = %d, msi_irqs = %d\n",
		 info->num_msi_host_irqs, info->num_msi_irqs);
#endif
	info->error_irq = irq_of_parse_and_map(node, info->num_legacy_irqs +
						info->num_msi_host_irqs);
	if (info->error_irq < 0) {
		pr_err(DRIVER_NAME ": No error irq defined\n");
		goto err;
	}

	return 0;
err:
	iounmap((void __iomem *)info->reg_cfg_virt);
	return err;
}


static int __init keystone_pcie_controller_init(struct device_node *np,
						int domain)
{
	struct keystone_pcie_info *rc_info;
	const struct of_device_id *of_id;
	int err = -EINVAL, i;
	struct clk *pcie_clk;
	int port = 0;

	pr_info(DRIVER_NAME ": keystone_pcie_rc_init - start\n");

	rc_info = kzalloc(sizeof(*rc_info), GFP_KERNEL);
	if (!rc_info) {
		pr_err(DRIVER_NAME ": Memory alloc failure\n");
		return -ENOMEM;
	}

	of_id = of_match_node(keystone_pci_match_ids, np);

	err = keystone_pcie_get_resources_all(np, rc_info);
	if (err < 0) {
		pr_err(DRIVER_NAME ": Unable to get resources\n");
		goto err1;
	}


err1:
	clk_put(pcie_clk);
err:
	kfree(rc_info);
	of_node_put(np);
	return err;
}


static int __init ti_k2h_pcie_resource(void)
{
	struct device_node *np = NULL;
	int ret = 0, domain = 0;


	for_each_matching_node(np, keystone_pci_match_ids) {
	printk("%s[%d]\n",__FUNCTION__,__LINE__);
		if (of_device_is_available(np))
		{
			printk("%s[%d]\n",__FUNCTION__,__LINE__);
			ret = keystone_pcie_controller_init(np, domain);
		}
		domain++;
	}
	return ret;
}


/**
 * ti_k2h_ep_pci_init() - init part of driver.
 *
 * get necessary resources.
 */

static int __init ti_k2h_ep_pcie_init(void)
{
	int ret = 0,i = 0;
	u32 temp;
	struct ti_k2h_bar_info bar_info[6];
	struct ti_k2h_inb_window in;
	struct ti_k2h_outb_region out;
	struct keystone_pcie_info *ep_info;
	const struct of_device_id *of_id;
	struct device_node *np;
	int err = -EINVAL;
	
	
/* !@@@X	if (resv_size > ti81xx_ep_mem_size) {
		pr_err(DRIVER_NAME ": can't reserve more memory than "
				"requested	at kernel cmd line ( pci_mem )\n");
		return -1;
	} else {
		ti81xx_edma_mem_start = ti81xx_ep_mem_start + resv_size;
		ti81xx_edma_mem_size = ti81xx_ep_mem_size - resv_size;
	}
	*/
	ret = alloc_chrdev_region(&ti_k2h_ep_pcie_dev, DEV_MINOR, DEVICENO,
							TI_K2H_PCIE_MODFILE);
	if (ret < 0) {

		pr_err(DRIVER_NAME ": could not allocate the "
							"character driver");
		return -1;
	}

	cdev_init(&ti_k2h_ep_pcie_cdev, &ti_k2h_ep_pcie_fops);
	ret = cdev_add(&ti_k2h_ep_pcie_cdev, ti_k2h_ep_pcie_dev, DEVICENO);
	if (ret < 0) {
		pr_err(DRIVER_NAME ": cdev add failed");
		unregister_chrdev_region(ti_k2h_ep_pcie_dev, DEVICENO);
		return -1;
	}
	
	/*PCIE Registart start Address 0x21800000*/
	reg_vir = (u32)ioremap_nocache(PCIE_REGS_START, PCIE_REGS_SIZE);
	if (!reg_vir) {
		pr_err(DRIVER_NAME ": register remapping failed");
		cdev_del(&ti_k2h_ep_pcie_cdev);
		unregister_chrdev_region(ti_k2h_ep_pcie_dev, DEVICENO);
		return -1;
	}

	temp = __raw_readl(reg_vir + LOCAL_CONFIG_OFFSET +VENDOR_DEVICE_ID);
	printk("the id is 0x%08x\n",temp);


	
	__raw_writel(2, reg_vir + OB_SIZE);
	temp = __raw_readl(reg_vir + OB_SIZE);
	printk("%s[%d]OB_SIZE[%d]\n",__FUNCTION__,__LINE__,temp);

	
	//ti_k2h_pcie_resource();
#if 1//set inbound region
	request_mem_region(PCIE_NON_PREFETCH_START,0x10000000, "pcie-nonprefetch-1");
	request_mem_region(PCIE_NON_PREFETCH_START + 0x10000000,0x400000, "pcie-nonprefetch-2");
	
	ks2reg_vir = (u32)ioremap_nocache(PCIE_NON_PREFETCH_START + 0x10000000, 0x400000);
	if (!ks2reg_vir) {
		pr_err(DRIVER_NAME ":request-ks2reg_vir failed in SharedRegion\n");
		iounmap((void *)reg_vir);
		cdev_del(&ti_k2h_ep_pcie_cdev);
		unregister_chrdev_region(ti_k2h_ep_pcie_dev, DEVICENO);
		return -1;
	}
	
	printk("%s[%d]\n",__FUNCTION__,__LINE__);
	for(i = 0;i < 6;i++)
	{
		bar_info[i].bar_num = i;
		get_bar_info(&bar_info[i]);
		printk("bar_num = %d,bar_addr = 0x%08x,bar_size=0x%08x\n",bar_info[i].bar_num,bar_info[i].bar_addr,bar_info[i].bar_size);
	}
	printk("%s[%d]\n",__FUNCTION__,__LINE__);
	for(i = 1;i < 4;i++)
	{
		in.BAR_num= i;
		printk("in.BAR_num=%d\n",in.BAR_num);
		if(in.BAR_num == 1)
		{
			in.internal_addr = PCIE_NON_PREFETCH_START + 0x10000000;
			in.ib_start_hi = 0;
			in.ib_start_lo = bar_info[i].bar_addr;
		}
		else if(in.BAR_num == 2)
		{
			in.internal_addr = PCIE_NON_PREFETCH_START + 0x08000000;
			in.ib_start_hi = 0;
			in.ib_start_lo = bar_info[i].bar_addr;
		}
		else if(in.BAR_num == 3)
		{
			in.internal_addr = PCIE_NON_PREFETCH_START + 0x00000000;
			in.ib_start_hi = 0;
			in.ib_start_lo = bar_info[i].bar_addr;
		}

		ti_k2h_set_inbound(&in);
	}
	
	
	temp = __raw_readl(reg_vir + CMD_STATUS);
	temp = temp | ENABLE_IN;
	__raw_writel(temp, reg_vir + CMD_STATUS);
	
#endif
	printk("%s[%d]\n",__FUNCTION__,__LINE__);

	
	temp = __raw_readl(reg_vir + IB_OFFSET(1));
	printk("memory register physical addr is 0x%08x\n",temp);
	/*Request 4MB Memory from 0xB0C00000 ~ 0xB1000000,Use to save info between L2 and PC*/
	mem_reg_vir = (u32)ioremap_nocache(temp, MEM_REG_SIZE);
	if (!mem_reg_vir) {
		pr_err(DRIVER_NAME ": register remapping failed");
		iounmap((void *)reg_vir);
		cdev_del(&ti_k2h_ep_pcie_cdev);
		unregister_chrdev_region(ti_k2h_ep_pcie_dev, DEVICENO);
		return -1;
	}
	
	temp = __raw_readl(mem_reg_vir);
	printk("mem_reg_vir is 0x%08x\n",temp);
	request_mem_region(PCIE_DATA_SPACE_START,256*1024*1024, "pcie-nonprefetch");

	



	ti_k2h_pcie_interrupt_set();
	INIT_WORK(&pcie_int_info.tx_wq, tx_wq_func);

	printk("%s[%d]\n",__FUNCTION__,__LINE__);
	ti_k2h_pci_class = class_create(THIS_MODULE, TI_K2H_PCIE_MODFILE);
	if (!ti_k2h_pci_class) {
		pr_err(DRIVER_NAME ":failed to add device to sys fs");
		goto init_err;
	}
	printk("%s[%d]\n",__FUNCTION__,__LINE__);
	device_create(ti_k2h_pci_class, NULL, ti_k2h_ep_pcie_dev, NULL,
						TI_K2H_PCIE_MODFILE);

	
#ifdef SUPPORT_PCIE_MSI
	ti_k2h_setup_msi();
#else

#endif

	printk("%s[%d]\n",__FUNCTION__,__LINE__);

//	for (i = 0; i < 4; i++) 
//	{
//		__raw_writel(0x1, reg_vir + LEGACY_N_IRQ_ENABLE_CLR(i));
//	}
//	__raw_writel(0x1, reg_vir + EP_IRQ_CLR);



	printk("%s[%d]\n",__FUNCTION__,__LINE__);
	INIT_LIST_HEAD(&outb_req);

	/*
	 * set all regions available
	 */

	status_out = 0x7ffffff0;
	//init_waitqueue_head(&readq);
	//sema_init(&sem_poll, 1);
	device_id = (__raw_readl(reg_vir + LOCAL_CONFIG_OFFSET +
						VENDOR_DEVICE_ID)) >> 16;
	printk("%s[%d]\n",__FUNCTION__,__LINE__);
	for(i = 0;i < 4;i ++)
	{
		temp = __raw_readl(reg_vir + IB_OFFSET(i));
		printk("IB_OFFSET = 0x%08x\n",temp);

		temp = __raw_readl(reg_vir + IB_START_HI(i));
		printk("IB_START_HI = 0x%08x\n",temp);

		temp = __raw_readl(reg_vir + IB_START_LO(i));
		printk("IB_START_LO = 0x%08x\n",temp);
		temp = __raw_readl(reg_vir + IB_BAR(i));
		printk("IB_BAR = 0x%08x\n",temp);
	}

	if (!ti_k2h_ep_mem_size)
		pr_warn(DRIVER_NAME ": *** WARNING: "
				"No memory reserved for PCIe transfers.\n\t"
				"Append pcie_mem<size> to kernel command line "
				"to use mmap/DMA on reserved memory.\n");
	
#ifdef SUPPORT_EDMA
    init_tx_queue();
	spin_lock_init(&tx_lock);
	sema_init(&tx_queue_sema, 0);
	tx_thread = kthread_run(tx_thread_func, NULL, "pcie_data_tx_thread");
	
	if(IS_ERR(tx_thread))
			printk( "tx_thread create fail\n");
#endif


	gpio_reg_va = ioremap_nocache(0x0260bf00, 0x38);

	if(gpio_reg_va == 0)
	goto init_err;
	
	keystone_get_l2_addr(0x20);
	printk( ": Initialization complete load successful\n");
	debug_thread = kthread_run(debug_thread_func, NULL, "debug_thread_func");

	__raw_writel(0x00000001, ks2reg_vir + L2CARD_STATUS);



	return 0;

	init_err:
	//iounmap((void *)reg_mem);
	iounmap((void *)reg_vir);
	iounmap((void *)mem_reg_vir);
	//iounmap((void *)res_mem);

/*!@@@X 	iounmap((void *)mgmt_area_remap);*/
	release_mem_region(PCIE_DATA_SPACE_START,
					PCIE_DATA_SPACE_SIZE);
	release_mem_region(PCIE_NON_PREFETCH_START,
					PCIE_DATA_SPACE_SIZE);
	cdev_del(&ti_k2h_ep_pcie_cdev);
	unregister_chrdev_region(ti_k2h_ep_pcie_dev, DEVICENO);
	pr_err(DRIVER_NAME ":failed to init pcie endpoint\n");
	return -1;
}



/**
 * ti_k2h_ep_pci_exit() - exit part of driver.
 *
 * release resources.
 */

static void __exit ti_k2h_ep_pcie_exit(void)
{
	int i = 0;
	
	for(i = 0;i<8;i++)
	free_irq(62+i, &ti_k2h_ep_pcie_dev);
	
	device_destroy(ti_k2h_pci_class, ti_k2h_ep_pcie_dev);
	class_destroy(ti_k2h_pci_class);
	
	//iounmap((void *)reg_mem);
	iounmap((void *)reg_vir);
	iounmap((void *)mem_reg_vir);
	//iounmap((void *)res_mem);
	iounmap((void *)gpio_reg_va);


/*!@@@X		iounmap((void *)mgmt_area_remap);*/
	release_mem_region(PCIE_DATA_SPACE_START, PCIE_DATA_SPACE_SIZE);
	release_mem_region(PCIE_NON_PREFETCH_START, PCIE_DATA_SPACE_SIZE);
	cdev_del(&ti_k2h_ep_pcie_cdev);
	unregister_chrdev_region(ti_k2h_ep_pcie_dev, DEVICENO);
	
	printk( ":module unload successful\n");
}


module_init(ti_k2h_ep_pcie_init);
module_exit(ti_k2h_ep_pcie_exit);
MODULE_LICENSE("GPL");

