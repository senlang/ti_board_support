#ifndef __PCIE_KEYSTONE_DRV_H
#define __PCIE_KEYSTONE_DRV_H
#include "pcie_keystone_types.h"



/**************************************************************************\
* Register Overlay Structure
PCIE capabilities, implementation specific registers
\**************************************************************************/
typedef struct  {
    volatile Uint32 PMCAP;
    volatile Uint32 PM_CTL_STAT;
    volatile Uint8 RSVD0[8];
    volatile Uint32 MSI_CAP;
    volatile Uint32 MSI_LOW32;
    volatile Uint32 MSI_UP32;
    volatile Uint32 MSI_DATA;
    volatile Uint8 RSVD1[16];
    volatile Uint32 PCIES_CAP;
    volatile Uint32 DEVICE_CAP;
    volatile Uint32 DEV_STAT_CTRL;
    volatile Uint32 LINK_CAP;
    volatile Uint32 LINK_STAT_CTRL;
    volatile Uint32 SLOT_CAP;
    volatile Uint32 SLOT_STAT_CTRL;
    volatile Uint32 ROOT_CTRL_CAP;
    volatile Uint32 ROOT_STATUS;
    volatile Uint32 DEV_CAP2;
    volatile Uint32 DEV_STAT_CTRL2;
    volatile Uint8 RSVD2[4];
    volatile Uint32 LINK_CTRL2;
    volatile Uint8 RSVD3[92];
    volatile Uint32 PCIE_EXTCAP;
    volatile Uint32 PCIE_UNCERR;
    volatile Uint32 PCIE_UNCERR_MASK;
    volatile Uint32 PCIE_UNCERR_SVRTY;
    volatile Uint32 PCIE_CERR;
    volatile Uint32 PCIE_CERR_MASK;
    volatile Uint32 PCIE_ACCR;
    volatile Uint32 HDR_LOG[4];
    volatile Uint32 RC_ERR_CMD;
    volatile Uint32 RC_ERR_ST;
    volatile Uint32 ERR_SRC_ID;
    volatile Uint8 RSVD4[1480];
    volatile Uint32 PL_ACKTIMER;
    volatile Uint32 PL_OMSG;
    volatile Uint32 PL_FORCE_LINK;
    volatile Uint32 ACK_FREQ;
    volatile Uint32 PL_LINK_CTRL;
    volatile Uint32 LANE_SKEW;
    volatile Uint32 SYM_NUM;
    volatile Uint32 SYMTIMER_FLTMASK;
    volatile Uint32 FLT_MASK2;
    volatile Uint8 RSVD5[4];
    volatile Uint32 DEBUG0;
    volatile Uint32 DEBUG1;
    volatile Uint8 RSVD6[220];
    volatile Uint32 PL_GEN2;
} PCIE_CAP_Implement_Regs;

typedef struct  {
    volatile Uint32 MSI_CAP;
    volatile Uint32 MSI_LOW32;
    volatile Uint32 MSI_UP32;
    volatile Uint32 MSI_DATA;
} PCIE_MSI_Regs;

typedef enum{
	PCIE_LOOPBACK_DISABLE = 0,
	PCIE_PHY_LOOPBACK 	/*PHY loopback is only vaild with RC mode*/
}PCIE_Loopback_Mode;

/** These are the possible values for PCIe mode */
typedef enum 
{
	PCIE_EP_MODE = 0,    
	PCIE_LEGACY_EP_MODE, 
	PCIE_RC_MODE   
}PCIE_Mode;

typedef enum{
	PCIE_ADDRESS_32_BITS = 0,
	PCIE_ADDRESS_64_BITS
}PCIE_Address_Width;

/*endian swap for PCIE access*/
typedef enum 
{
	PCIE_ENDIAN_SWAP_ON_1_BYTE = 0,    
	PCIE_ENDIAN_SWAP_ON_2_BYTES, 
	PCIE_ENDIAN_SWAP_ON_4_BYTES,
	PCIE_ENDIAN_SWAP_ON_8_BYTES
}PCIE_Endian_Swap;

/*PCIE outbound region size*/
typedef enum 
{
	PCIE_OB_SIZE_1MB= 0,
	PCIE_OB_SIZE_2MB,
	PCIE_OB_SIZE_4MB,
	PCIE_OB_SIZE_8MB
}PCIE_Outbound_Region_Size;

/*LTSSM STATE*/
typedef enum 
{
	LTSSM_STAT_DETECT_QUIET      = 0x00,
	LTSSM_STAT_DETECT_ACT        = 0x01,
	LTSSM_STAT_POLL_ACTIVE       = 0x02,
	LTSSM_STAT_POLL_COMPLIANCE   = 0x03,
	LTSSM_STAT_POLL_CONFIG       = 0x04,
	LTSSM_STAT_PRE_DETECT_QUIET  = 0x05,
	LTSSM_STAT_DETECT_WAIT       = 0x06,
	LTSSM_STAT_CFG_LINKWD_START  = 0x07,
	LTSSM_STAT_CFG_LINKWD_ACEPT  = 0x08,
	LTSSM_STAT_CFG_LANENUM_WAIT  = 0x09,
	LTSSM_STAT_CFG_LANENUM_ACEPT = 0x0A,
	LTSSM_STAT_CFG_COMPLETE      = 0x0B,
	LTSSM_STAT_CFG_IDLE          = 0x0C,
	LTSSM_STAT_RCVRY_LOCK        = 0x0D,
	LTSSM_STAT_RCVRY_SPEED       = 0x0E,
	LTSSM_STAT_RCVRY_RCVRCFG     = 0x0F,
	LTSSM_STAT_RCVRY_IDLE        = 0x10,
	LTSSM_STAT_L0                = 0x11,
	LTSSM_STAT_L0s               = 0x12,
	LTSSM_STAT_L123_SEND_EIDLE   = 0x13,
	LTSSM_STAT_L1_IDLE           = 0x14,
	LTSSM_STAT_L2_IDLE           = 0x15,
	LTSSM_STAT_L2_WAKE           = 0x16,
	LTSSM_STAT_DISABLED_ENTRY    = 0x17,
	LTSSM_STAT_DISABLED_IDLE     = 0x18,
	LTSSM_STAT_DISABLED          = 0x19,
	LTSSM_STAT_LPBK_ENTRY        = 0x1A,
	LTSSM_STAT_LPBK_ACTIVE       = 0x1B,
	LTSSM_STAT_LPBK_EXIT         = 0x1C,
	LTSSM_STAT_LPBK_EXIT_TIMEOUT = 0x1D,
	LTSSM_STAT_HOT_RESET_ENTRY   = 0x1E,
	LTSSM_STAT_HOT_RESET         = 0x1F
}PCIE_LTSSM_State;

/*the memory region be accessed through PCIE.*/
typedef struct{
	Uint32 uiTargetAddress;//the address in target memory
	Uint32 uiNumBytes;
}PCIE_Memory_Region;

/*multiple memory regions with same prefetchable property be accessed 
through PCIE, these regions may be mapped to same BAR*/
typedef struct{
	PCIE_Memory_Region * memory_regions;
	Uint32 uiNumRegions;
	Bool   bPrefetchable;

	/*below are not input parameters. These are calculated during 
	initialization according to the size of all regions*/
	unsigned long long ullTotalSize;/*total size of the memory regions*/
	unsigned long long ullBAR_Mask; /*BAR mask for corresponding BAR*/
}PCIE_Memory_Regions;

/*multiple memory regions be accessed through PCIE inbound interface.
Please note, number of inbound memory regions can not be greater than 4.
In RC mode, there is only one memory BAR with 32-bit address.*/
typedef struct{
	//prefetchable memory regions (will be mapped to one BAR)
	PCIE_Memory_Regions * prefetch_regions; 

	//non-prefetchable memory regions (will be mapped to one BAR)
	PCIE_Memory_Regions * nonfetch_regions;
}PCIE_Inbound_Memory_Regions;

/*PCIE outbound regions configure*/
typedef struct{
	/*uiNumRegions and address_offset are optional,
	if they are NULL, RC will configure outbound memory via enumeration.*/
	unsigned long long * address_offset;
	Uint32 uiNumRegions; 

	PCIE_Outbound_Region_Size OB_size;
}PCIE_Outbound_Memory_Regions;

/*PCIE BAR configure*/
typedef struct{
	unsigned long long ullMask; //Mask represent size of the BAR
	PCIE_Address_Width address_width;
	Bool   bPrefetchable;
	Bool   bIO;	//is IO space BAR

	unsigned long long ullStart_address;
}PCIE_BAR_Config;

/*Remote Configuration Transaction Setup,
select the bus, device and function number of the target*/
typedef struct
{
	Uint8 config_type; //type 0 is EP; type 1 is RC
	Uint8 config_bus;
	Uint8 config_device;
	Uint8 config_function;
}PCIE_Remote_CFG_SETUP;

/*PCIE RC mode specific configuration*/
typedef struct{
	/*the space inside (base, limit) range are for EP,
	access outside of this range will go to RC*/
	Uint32 memory_base;
	Uint32 memory_limit;

	/*Though this parameter is 64-bit, the higher 32 bits are 
	ignored in 32-bit mode.*/
	unsigned long long prefetch_memory_base;
	unsigned long long prefetch_memory_limit;

	/*RC BAR0 directly map to RC's PCIE application registers*/
	Uint32 BAR0_address;
}PCIE_RC_Config;

/*The configuration for internal bus between PCIE and memory subsystem*/
typedef struct{
	Bool bSupervisor; //PCIE has supervisor privilege on memory subsystem
	Uint8 priority;

	/*endian swap for PCIE access*/
	PCIE_Endian_Swap endian_swap;	
}PCIE_Internal_Bus_Config;

/*number of MSI vectors need for this EP*/
typedef enum 
{
	PCIE_1_MSI=0,
	PCIE_2_MSI,
	PCIE_4_MSI,
	PCIE_8_MSI,
	PCIE_16_MSI,
	PCIE_32_MSI,
	PCIE_NO_TX_MSI 	/*no MSI is generated from this EP*/
}PCIE_number_MSI;

/*interrupt configuration*/
typedef struct{
	
	Uint32 MSI_rx_enable_mask; /*each bit enable reception of one MSI*/
	Bool   Err_rx_enable; 	/*enable error interrupt reception*/
	Bool   PMRST_rx_enable; 	/*enable power and reset interrupt reception*/
	PCIE_number_MSI number_tx_MSI; /*number of MSI may generate from this EP*/
}PCIE_Interrupt_Config;

/*PCIE Error check, dectction, report configuration*/
typedef struct{
	Bool bErrorEnable;
	
	//to do
}PCIE_Error_Config;

typedef struct  {
	PCIE_Mode PcieMode;
	PCIE_Loopback_Mode loop_mode;
	PCIE_Address_Width address_width;

	/*pointer to inbound memory configuration structure,
	NULL menas inbound access is not used*/
	PCIE_Inbound_Memory_Regions * inbound_memory_regions;

	/*pointer to outbound memory configuration structure.
	If it is NULL, in EP mode, menas outbound access is not used; in RC mode, 
	the OB size in this configuration structure must be provided*/
	PCIE_Outbound_Memory_Regions * outbound_memory_regions;

	/*pointer to PCIE RC mode specific configuration structure.
	For EP mode, assign NULL pointer to it*/
	PCIE_RC_Config * rc_cfg;

	Uint8 num_lanes;		//number of lanes
	Bool  bPcieGen2;		//enable PCIE GEN2 at 5Gbps

	/*this device and the device at the opposite end of the link are operating 
	with a common clock source.*/
	Bool bCommon_clock;

	/*pointer to the MSI interrupt configuration structure. 
	NULL pointer means MSI is not used*/
	PCIE_Interrupt_Config * interrupt_cfg;

	/*pointer to configuration structure for internal bus between PCIE and memory subsystem. 
	NULL pointer results in using default values*/
	PCIE_Internal_Bus_Config * bus_cfg;

	/*pointer to the PCIE Error check, dectction, report configuration structure. 
	NULL pointer results in using default values*/
	PCIE_Error_Config * error_cfg;

} K2_PCIE_Config;

/** @addtogroup CSL_SERDES
 @{ */
/** ============================================================================
 *   @n@b CSL_SERDES_REF_CLOCK
 *
 * @brief
 *
 *
 *  SERDES REF CLOCK speed enumerators */
typedef enum
{
    /** 100 MHz */
   CSL_SERDES_REF_CLOCK_100M        =   0,

    /** 122.8 MHz */
   CSL_SERDES_REF_CLOCK_122p88M     =   1,

    /** 125 MHz */
   CSL_SERDES_REF_CLOCK_125M        =   2,

    /** 153.6 MHz */
   CSL_SERDES_REF_CLOCK_153p6M     =    3,

   /** 156.25 MHz */
   CSL_SERDES_REF_CLOCK_156p25M     =   4,

    /** 312.5 MHz */
   CSL_SERDES_REF_CLOCK_312p5M      =   5
} CSL_SERDES_REF_CLOCK;

typedef enum
{
    /** Loopback Enabled */
    CSL_SERDES_LOOPBACK_ENABLED   =  0,

    /** Loopback Disabled */
    CSL_SERDES_LOOPBACK_DISABLED  =  1
} CSL_SERDES_LOOPBACK;

typedef struct
{
	CSL_SERDES_REF_CLOCK inputRefClock;

	float linkSpeed_GHz; 	/*the max link speed of all lanes at GHz*/

	/*number of lanes be used*/
	Uint32 numLanes; 

	CSL_SERDES_LOOPBACK  loopBackMode;

} K2_PCIE_SerdesConfig;


/* the "expression" macros */

/* the Field MaKe macro */
#define CSL_FMK(PER_REG_FIELD, val)                                         \
    (((val) << CSL_##PER_REG_FIELD##_SHIFT) & CSL_##PER_REG_FIELD##_MASK)

/* the Field EXTract macro */
#define CSL_FEXT(reg, PER_REG_FIELD)                                        \
    (((reg) & CSL_##PER_REG_FIELD##_MASK) >> CSL_##PER_REG_FIELD##_SHIFT)

/* the Field INSert macro */
#define CSL_FINS(reg, PER_REG_FIELD, val)                                   \
    ((reg) = ((reg) & ~CSL_##PER_REG_FIELD##_MASK)                          \
    | CSL_FMK(PER_REG_FIELD, val))


/* the "token" macros */

/* the Field MaKe (Token) macro */
#define CSL_FMKT(PER_REG_FIELD, TOKEN)                                      \
    CSL_FMK(PER_REG_FIELD, CSL_##PER_REG_FIELD##_##TOKEN)

/* the Field INSert (Token) macro */
#define CSL_FINST(reg, PER_REG_FIELD, TOKEN)                                \
    CSL_FINS((reg), PER_REG_FIELD, CSL_##PER_REG_FIELD##_##TOKEN)


/* the "raw" macros */

/* the Field MaKe (Raw) macro */
#define CSL_FMKR(msb, lsb, val)                                             \
    (((val) & ((1 << ((msb) - (lsb) + 1)) - 1)) << (lsb))

/* the Field EXTract (Raw) macro */
#define CSL_FEXTR(reg, msb, lsb)                                            \
    (((reg) >> (lsb)) & ((1 << ((msb) - (lsb) + 1)) - 1))

/* the Field INSert (Raw) macro */
#define CSL_FINSR(reg, msb, lsb, val)                                       \
    ((reg) = ((reg) &~ (((1 << ((msb) - (lsb) + 1)) - 1) << (lsb)))         \
    | CSL_FMKR(msb, lsb, val))


#define pcie_setbits(newval,field,val)                        \
	  { 														  \
		/* Eval "val" only once */								  \
		uint32_t working_val = val; 							  \
		uint32_t working_mask = (field##_MASK >> field##_SHIFT);  \
		/* warning if the value is outside the range */ 		  \
		/* This generates runtime overhead if pcie_DEBUG set */ \
		working_val &= working_mask;							  \
		working_val <<= field##_SHIFT;							  \
		newval &= ~field##_MASK;								  \
		newval |= working_val;									  \
	  } 
	
	
	/* Extracts a bitfield */
#define pcie_getbits(val,field,final_result) \
	  final_result = (val & field##_MASK) >> field##_SHIFT;

#endif
