/**
 * This file contains register offsets for TI816X PCIe Endpoint. Also
 * provides basic ioctls to user space and data structures used in driver.
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

#ifndef __TI_K2H_PCIE_EP_DRIVER__
#define __TI_K2H_PCIE_EP_DRIVER__

#define u32 unsigned int

/* ioctls defined for driver as well as user space */

#define TI_K2H_SET_INBOUND			_IOW('P', 3, struct ti_k2h_inb_window)
#define TI_K2H_SET_OUTBOUND_SIZE	_IOW('P', 4, unsigned int)
#define TI_K2H_SET_OUTBOUND			_IOW('P', 5, struct ti_k2h_outb_region)
#define TI_K2H_ACCESS_REGS			_IOWR('P', 6, struct ti_k2h_pciess_regs)
#define TI_K2H_SEND_MSI				_IOW('P', 7, struct ti_k2h_msi_bar)
#define TI_K2H_GET_PCIE_MEM_INFO	_IOR('P', 8, struct ti_k2h_pcie_mem_info)

#define TI_K2H_CUR_TIME             _IOR('p', 10, unsigned int)
#define TI_K2H_GET_OUTBOUND_STATUS	_IOR('p', 11, unsigned int)
#define TI_K2H_CLR_OUTBOUND_MAP		_IOW('p', 12, struct ti_k2h_outb_region)
#define TI_K2H_GET_INTR_CNTR	    _IOR('p', 13, unsigned int)
#define TI_K2H_GET_BAR_INFO			_IOWR('P', 14, struct ti_k2h_bar_info)
#define TI_K2H_SEND_INTX			_IOW('P', 15, struct ti_k2h_msi_bar)
#define TI_K2H_TRIGGER_IPCGRX		_IOW('P', 16, unsigned int)
#define TI_K2H_ACCESS_MEM_REG		_IOW('P', 17, unsigned int)
#define TI_K2H_SET_OUTBOUND_STATUS	_IOW('p', 18, unsigned int)


/* XXX: to be removed */

/**
 * ti_k2h_inb_window - inbound window configuration information
 * @BAR_num: bar no for which inbound window has to be mapped
 * @internel_addr: address to be set in translation register
 * (internal physical address)
 * @ib_start_hi: PCIe address higher if 64bit is used (will be zero for 32 bit)
 * @ib_start_lo: PCIe address (represents lower if 64bit is used)
 *
 * this structure contain information about inbound address translation setup
 */


struct ti_k2h_inb_window {
	u32 BAR_num;
	u32 internal_addr;
	u32 ib_start_hi;
	u32 ib_start_lo;
};

/**
 * ti_k2h_outb_region - outbound mapping related configuration
 * @ob_offset_hi: PCIe high address of 64 bit (will be zero for 32 bit)
 * @ob_offset_idx: PCIe address (represents lower if 64bit is used)
 * @size: size of mapping
 *
 * this structure contain information about outbound address translation setup
 */

struct ti_k2h_outb_region {
	u32 ob_offset_hi;
	u32 ob_offset_idx;
	u32 size;
};


/**
 * ti_k2h_pciess_regs - pciess register access information
 * @offset: offset of register from 0x51000000
 * @value: value to set in SET mode, and will be fill by driver in GET mode
 * @mode: get value or set value of register (SET_PCIE_REG/GET_PCIE_REG)
 *
 * this structure contain information about pciess register, we can use provided
 * ioctl
 * for debugging purpose as well as to see the register DUMP
 */


struct ti_k2h_pciess_regs {
	u32 offset;
	u32 value;
	u32 mode;
};

/**
 * ti_k2h_msi_info: MSI interuupt related information
 * @msi_data: data to be written in memory write transaction
 * @msi_addr_low: lower 32 bit of address for memory write transaction
 * @msi_addr_hi: higher 32 bit of address for memory write transaction
 *
 * this structure contain information about MSI interrupt generation
 */

struct ti_k2h_msi_info {
	u32 msi_data;
	u32 msi_addr_low;
	u32 msi_addr_hi;
};


/**
 * ti_k2h_pcie_mem_info: Contains information about the PCIe reserved memory
 * @base: Physical address of base of memory reserved
 * @psize: Size in bytes of the reserved memory
 */

struct ti_k2h_pcie_mem_info {
	u32 base;
	u32 size;
};

/**
 * ti_k2h_msi_bar: this structure contains bar0 and bar1 address passed
 * by application to genrate msi.
 * @bar0: address of bar0
 * @bar1: address of bar1
 *
 * in case 64 bit msi application have to pass both bar0 and bar1 while
 * in case of 32 bit msi
 * bar1 will always be passed as zero.
 */

struct ti_k2h_msi_bar {
	u32 bar0;
	u32 bar1;
};

/**
 * ti_k2h_bar_info: this structure is used to query bar info
 * @bar_addr: address allocated to bar
 * @bar_size: size of bar
 * @bar_num: bar no for which query is sent
 */

struct ti_k2h_bar_info {
	u32 bar_addr;
	u32 bar_addr_HI;
	u32 bar_size;
	u32 bar_num;
};
#endif
