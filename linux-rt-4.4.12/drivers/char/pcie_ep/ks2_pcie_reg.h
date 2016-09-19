#ifndef _KS2_PCIE_REG_H
#define _KS2_PCIE_REG_H

/* offsets of application registers  starting from 0x51000000 */
#define CMD_STATUS                      0x004
#define CFG_SETUP                       0x008
#define IB_BAR(x)                       (0x300 + (0x10 * x))
#define IB_START_LO(x)                  (0x304 + (0x10 * x))
#define IB_START_HI(x)                  (0x308 + (0x10 * x))
#define IB_OFFSET(x)                    (0x30c + (0x10 * x))
#define OB_SIZE                         0x30
#define ENDIAN							0x38
#define PRIORITY						0x3c
#define CFG_PCIM_WIN_CNT                32
#define CFG_PCIM_WIN_SZ_IDX             3
#define OB_OFFSET_INDEX(n)              (0x200 + (8 * n))
#define OB_OFFSET_HI(n)                 (0x204 + (8 * n))
#define MSI_IRQ                         0x54
#define MSI0_IRQ_STATUS                 0x104
#define MSI0_IRQ_ENABLE_SET             0x108
#define MSI0_IRQ_ENABLE_CLR             0x10c

#define MSIX_IRQ(n)						(0x100 + (0x10 * n))
#define MSI_IRQ_STATUS_RAW				0
#define MSI_IRQ_STATUS					0x04
#define MSI_IRQ_ENABLE_SET             	0x08
#define MSI_IRQ_ENABLE_CLR             	0x0c

#define LEGACY_N_IRQ_STATUS_RAW(n)      (0x180 + (0x10*(n)))
#define LEGACY_N_IRQ_ENABLE_SET(n)      (0x188 + (0x10*(n)))
#define LEGACY_N_IRQ_ENABLE_CLR(n)      (0x18c + (0x10*(n)))


#define IRQ_EOI                         0x50
#define GPR0				0x70

/* these three application register are for alternate mechanism
 * to send interrupt from ep to rc
 */

#define EP_IRQ_SET                      0x64
#define EP_IRQ_CLR                      0x68
#define EP_IRQ_STATUS                   0x6c

/* these are for monitoring error status */
#define ERR_IRQ_STATUS_RAW              0x1c0
#define ERR_IRQ_STATUS                  0x1C4
#define ERR_IRQ_ENABLE_SET              0x1C8
#define ERR_IRQ_ENABLE_CLR              0x1CC

#define PMRST_IRQ_STATUS_RAW              0x1D0
#define PMRST_IRQ_STATUS                  0x1D4
#define PMRST_IRQ_ENABLE_SET              0x1D8
#define PMRST_IRQ_ENABLE_CLR              0x1DC



/*configuration register these are at offset 0x1000 from 0x21800000*/
#define STATUS_COMMAND                  0x4
#define VENDOR_DEVICE_ID                0x0
#define  BAR0                           0x10
#define  BAR1                           0x14
#define  BAR2                           0x18
#define  BAR3                           0x1C
#define BARn(n)							(0x10 + (4*n))
/* msi capability registers-- these register are at 0x1000+0x50
 * from 0x21800000
 */

#define MSI_CAP                         0x0
#define MSI_LOW32                       0x4
#define MSI_UP32                        0x8
#define MSI_DATA                        0xC

#define MSI_OFF                         0x50

/* power management capability register-- thesew are at offset
 * 0x1000+0x40 from 0x51000000
 */

#define  PMCAP                          0x0
#define  PM_CTL_STAT                    0x4
/* add registers for error lokkup*/

/* other general macro related to driver*/

#define DEV_MINOR			0
#define DEVICENO			1
#define DRIVER_NAME			"ti_k2h_ep_pcie"

#define PCIE_INT0 26	//legacy INTA interrupt
#define PCIE_INT1 27	//legacy INTB interrupt
#define PCIE_INT2 28	//legacy INTB interrupt
#define PCIE_INT3 29	//legacy INTD interrupt
#define PCIE_INT4 30	//PCIE MSI interrupt
#define PCIE_INT5 31	//PCIE MSI interrupt
#define PCIE_INT6 32	//PCIE MSI interrupt
#define PCIE_INT7 33	//PCIE MSI interrupt
#define PCIE_INT8 34	//PCIE MSI interrupt
#define PCIE_INT9 35	//PCIE MSI interrupt
#define PCIE_INT10 36	//PCIE MSI interrupt
#define PCIE_INT11 37	//PCIE MSI interrupt
#define PCIE_INT12 38	//PCIE error interrupt
#define PCIE_INT13 39	//PCIE power management interrupt

#define LOCAL_CONFIG_OFFSET		0x1000
#define ENABLE_MASTER	0x4

#define CFG_PCIM_WIN_CNT		32
#define CFG_PCIM_WIN_SZ_IDX		3

#define PCIE_NON_PREFETCH_START		0xC0000000//0xB0000000//0xA3000000
#define PCIE_NON_PREFETCH_SIZE		(128 * 1024 * 1024)

//PCIE_PREFETCH_BASE_ADDRESS

#define PCIE_DATA_SPACE_START		0x50000000
#define PCIE_DATA_SPACE_SIZE		(256 * 1024 * 1024)

#define PCIE_REGS_START			0x21800000
#define PCIE_REGS_SIZE			(16 * 1024 * 1024)

#define PCIE_DEVCFG				0x0262014c	//20151010 add
#define PCIE_DEVCFG_SIZE		(4)			//20151010 add
#define PCIE_SERDESCFG			0x02320000	//20151010 add	
#define PCIE_SERDESCFG_SIZE		(16 * 1024)	//20151010 add

#define IPCGR_REG_START			0x02620240		//20151225 add
#define IPCGR_REG_SIZE		0x70
#define IPCGR0				0x0
#define IPCAR0				0x40

#define MEM_REG_START		PCIE_NON_PREFETCH_START + 0x10000000		//20160418 add
#define MEM_REG_SIZE		(4 * 1024 * 1024)



#define TI_K2H_PCIE_MODFILE		"ti_k2h_pcie_ep"

#define TI816X_PCI_VENDOR_ID		0x104c
#define TI816X_PCI_DEVICE_ID		0xb800
#define SET_REGS			1
#define GET_REGS			2
#define MB				(1024 * 1024)
#define NMEM				1
#define INOB				2
#define FAIL				3
#define XCEEDREG			32

/*{ //20151010 add start*/
#define PCIE_REGS_INDEX			0
#define	PCIE_NON_PREFETCH_INDEX		1
#define PCIE_IO_INDEX			2
#define PCIE_INBOUND0_INDEX		3

/*
 *  Application Register Offsets
 */
#define PCISTATSET			0x010
//#define CMD_STATUS			0x004
//#define CFG_SETUP			0x008
#define IOBASE				0x00c
//#define OB_SIZE				0x030
//#define IRQ_EOI                         0x050
//#define MSI_IRQ				0x054
/* 32 Registers */
#define OB_OFFSET_INDEX(n)		(0x200 + (8 * n))
/* 32 Registers */
#define OB_OFFSET_HI(n)			(0x204 + (8 * n))
#define IB_BAR0				0x300
#define IB_START0_LO			0x304
#define IB_START0_HI			0x308
#define IB_OFFSET0			0x30c
#define ERR_IRQ_STATUS_RAW		0x1c0
//#define ERR_IRQ_STATUS			0x1c4
//#define ERR_IRQ_ENABLE_SET		0x1c8
//#define MSI0_IRQ_STATUS			0x104
//#define MSI0_IRQ_ENABLE_SET		0x108
//#define MSI0_IRQ_ENABLE_CLR		0x10c
#define IRQ_STATUS			0x184
#define IRQ_ENABLE_SET			0x188
#define IRQ_ENABLE_CLR			0x18c

#define MSI_IRQ_OFFSET			4

/*
 * PCIe Config Register Offsets (capabilities)
 */
#define LINK_CAP		        0x07c
#define LINK_STAT_CTRL			0x80
#define PCI_STATUS		0x06	/* 16 bits */

/*
 * PCIe Config Register Offsets (misc)
 */
#define PL_FORCE_LINK			0x708
#define DEBUG0			        0x728
#define PL_GEN2			        0x80c

/* Various regions in PCIESS address space */
#define SPACE0_LOCAL_CFG_OFFSET		0x1000
#define SPACE0_REMOTE_CFG_OFFSET	0x2000
#define SPACE0_IO_OFFSET		0x3000

/* Application command register values */
#define DBI_CS2_EN_VAL		        BIT(5)
#define IB_XLAT_EN_VAL		        BIT(2)
#define OB_XLAT_EN_VAL		        BIT(1)
#define LTSSM_EN_VAL		        BIT(0)

/* Link training encodings as indicated in DEBUG0 register */
#define LTSSM_STATE_MASK	        0x1f
#define LTSSM_STATE_L0		        0x11

/* Directed Speed Change */
#define DIR_SPD				(1 << 17)

/* Outbound window size specified as power of 2 MB */
#define CFG_PCIM_WIN_SZ_IDX	        3
#define CFG_PCIM_WIN_CNT	        32

/* max 1GB DMA range */
#define MAX_DMA_RANGE			0x80000000
#define MAX_LEGACY_HOST_IRQS		4
#define MAX_MSI_IRQS			32
#define MAX_MSI_HOST_IRQS		8

#define ENABLE_IN	0x4
#define ENABLE_OUT	0x2

/* error IRQ bits */
#define ENABLE_ERR_FATAL_IRQ		BIT(1)
#define ERR_FATAL_IRQ			BIT(1)
#define pcie_debug	printk
/*//20151010 add end }*/

#define reg_dump(addr, mask) \
		pr_debug("reg %p has value %x\n", (void *)addr, \
				(__raw_readl(addr) & ~mask))


#define PCIE_EP_MODE		(0)
#define PCIE_RC_MODE		(BIT(2))
#define PCIE_MODE_MASK		(BIT(1) | BIT(2))

/* mask bits point to bits being modified */
#define reg_rmw(addr, value, mask) \
	__raw_writel(((__raw_readl(addr) & (~(mask))) | \
			(value & (mask))), (addr))

/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////
//开发板使用
#define KS2_START_ADDR		0x0
#define BAR_START_LO(n)		(KS2_START_ADDR + 0x0 + (0x10 * (n)))//层二板BAR0 起始地址
#define BAR_START_HI(n)		(KS2_START_ADDR +0x4 + (0x10 * (n))	)//层二板BAR0 起始地址
#define BAR_SIZE(n)			(KS2_START_ADDR +0x8 + (0x10 * (n)))

#define BUFF_PA_LO 	(KS2_START_ADDR + 0x60)	//BufferPhysicalAddress,驱动所分配的用于层二板往PC 写数据
#define BUFF_PA_HI 	(KS2_START_ADDR + 0x64)	//BufferPhysicalAddress,驱动所分配的用于层二板往PC 写数据
#define BUFF_SIZE 	(KS2_START_ADDR + 0x68) //BufferPhysicalAddress,驱动所分配的用于层二板往PC 写数据
#define BUFF_PA_END (KS2_START_ADDR + 0x6c)		//BufferPhysicalAddress,驱动所分配的用于层二板往PC 写数据


#define WRITE_START_ADDR_LO	(KS2_START_ADDR + 0x70)
#define READ_START_ADDR_LO	(KS2_START_ADDR + 0x74)

#define WRITE_END_ADDR_LO	(KS2_START_ADDR + 0x80)
#define WRITE_LOOP_COUNT	(KS2_START_ADDR + 0x84)
#define READ_END_ADDR_LO	(KS2_START_ADDR + 0x88)
#define READ_LOOP_COUNT		(KS2_START_ADDR + 0x8C)



#define L1CARD_NUM							(KS2_START_ADDR + 0x100)
#define L1CARD_BAR0_LO(n)					(KS2_START_ADDR + 0x110 + (0x10 * (n)))//层一板BAR0 起始地址
#define L1CARD_BAR0_SIZE(n)					(KS2_START_ADDR + 0x114 + (0x10 * (n)))
#define L1CARD_BAR0_PCIE_OB_ADDR(n)			(KS2_START_ADDR + 0x118 + (0x10 * (n)))//层一板BAR0 映射到层二板PCIE Data 地址
#define L1CARD_BAR0_PCIE_OB_OFFSET(n)		(KS2_START_ADDR + 0x11C + (0x10 * (n)))//层一板BAR0 在PCIE Data 中的偏移



#define PCIE_SRC_ADDR(n)					(KS2_START_ADDR + 0x300 + (0x10 * (n)))/*DSP 发送数据的源地址*/
#define PCIE_DEST_ADDR(n)					(KS2_START_ADDR + 0x304 + (0x10 * (n)))/*DSP 发送数据的源地址*/
#define PCIE_SIZE(n)						(KS2_START_ADDR + 0x308 + (0x10 * (n)))/*DSP 发送数据的长度*/


//#define  L3MEM_REG_LEAF_HEAD_PTR				(KS2_START_ADDR + 0x400 + 0x00)	// 接收BD链表的首地址:物理地址
//#define	 L3MEM_REG_DATA_BUFF_HEAD_PTR			(KS2_START_ADDR + 0x400 + 0x04)	// 接收DataBuffer的首地址：物理地址
//#define  L3MEM_REG_DATA_BUFF_SIZE				(KS2_START_ADDR + 0x400 + 0x08)	// 接收每个BD链表指向的DataBuffer的大小：32DW-固定
//#define  L3MEM_REG_LEAF_COUNT					(KS2_START_ADDR + 0x400 + 0x0C)	// 接收BD节点链表的数量
//#define  L3MEM_REG_FILLED_LEAF_COUNT			(KS2_START_ADDR + 0x400 + 0x10)	// 节点完成数量，由DMA回写:数据采集完成的数据块数量,循环计数，
//                                                    // 最大值不超过REG_LEAF_COUNT配置的节点数
//#define  L3MEM_REG_MAC_PACKET_LEN               (KS2_START_ADDR + 0x400 + 0x14)	// TLP帧净荷长度(DWORD) 初始值为32DW-固定不能修改
//#define  L3MEM_REG_FILLED_FRAME_LOW_CNT         (KS2_START_ADDR + 0x400 + 0x18)	// BMD链表接收转圈次数低32BIT
//#define  L3MEM_REG_FILLED_FRAME_HIGH_CNT        (KS2_START_ADDR + 0x400 + 0x1C)	// BIT[30:0]-BMD链表接收转圈次数高31BIT;
//                                                    // BIT[31]-BMD链表转圈控制，1-永久，0-由0x5C和0x60的配置值决定
//#define  L3MEM_REG_READ_LEAF_COUNT              (KS2_START_ADDR + 0x400 + 0x20)  // 节点读偏移，由Host回写，循环计数，最大值不超过REG_LEAF_COUNT配置的节点数


#define L3MEM_START_ADDR		(KS2_START_ADDR + 0x400 + 0x00)//主控板提供的共享内存的起始地址
#define L3MEM_SIZE				(KS2_START_ADDR + 0x400 + 0x04)//主控板提供的共享内存的大小
#define L3MEM_BUF_COUNT			(KS2_START_ADDR + 0x400 + 0x08)//DataBuffer个数，L3MEM_SIZE/128,每一个DataBuffer size为128Bytes
#define L3MEM_WRITE_INDEX		(KS2_START_ADDR + 0x400 + 0x0C)//写指针位置
#define L3MEM_READ_INDEX		(KS2_START_ADDR + 0x400 + 0x10)//读指针位置
#define L3MEM_DATA_BUFF_SIZE	(KS2_START_ADDR + 0x400 + 0x14)//Data buffer size
#define L3MEM_PCIE_OB_ADDR		(KS2_START_ADDR + 0x400 + 0x18)//L3 Memory mapping 到层二板pcie data space


#define L2MEM_START_ADDR		(KS2_START_ADDR + 0x420 + 0x00)//层二提供给主控板数据传输起始地址
#define L2MEM_SIZE				(KS2_START_ADDR + 0x420 + 0x04)//层二提供给主控板数据传输大小
#define L2MEM_BUF_COUNT			(KS2_START_ADDR + 0x420 + 0x08)//DataBuffer个数，L2MEM_SIZE/128,每一个DataBuffer size为128Bytes
#define L2MEM_WRITE_INDEX		(KS2_START_ADDR + 0x420 + 0x0C)//写指针位置
#define L2MEM_READ_INDEX		(KS2_START_ADDR + 0x420 + 0x10)//读指针位置
#define L2MEM_DATA_BUFF_SIZE	(KS2_START_ADDR + 0x420 + 0x14)//Data buffer size
#define L2MEM_PCIE_ADDR			(KS2_START_ADDR + 0x420 + 0x18)//



#define L2CARD_STATUS			(KS2_START_ADDR + 0x500 + 0x00)//层二板状态
#define L2CARD_CONTROL			(KS2_START_ADDR + 0x500 + 0x04)//层二板状态


#endif
