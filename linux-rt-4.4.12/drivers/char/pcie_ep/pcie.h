/*
 *
 * Copyright (C) 2010-2013 Texas Instruments Incorporated - http://www.ti.com/ 
 * 
 * 
 *  Redistribution and use in source and binary forms, with or without 
 *  modification, are permitted provided that the following conditions 
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the   
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/

#ifndef _PCIE_H
#define _PCIE_H

#ifdef __cplusplus
extern "C" {
#endif

/* System level header files */
#include "pcie_keystone_types.h"


/* ============================================================== */
/**
 *   @file  pcie.h
 *
 *   path  ti/drv/pcie/pcie.h
 *
 *   @brief PCIe sub-system API and Data Definitions
 *
 */
 
/**  @mainpage PCIe Low Level Driver
 *
 *   @section intro  Introduction
 *
 *    
 *   This document describes the Low Level Driver (LLD) for the Peripheral Component Interconnect Express (PCIe).
 *
 *   The PCI Express module supports dual operation mode: End Point (EP or Type0) or Root Complex (RC or Type1).
 *   This driver focuses on EP mode but it also provides access to some basic RC configuration/functionalities.
 *
 *   The PCIe userguide can be found at <http://www.ti.com/lit/sprugs6a>.
 *   
 *   The PCIe subsystem has two address spaces. The first (Address Space 0)
 *   is dedicated for local application registers, local configuration accesses and remote
 *   configuration accesses. The second (Address Space 1) is dedicated for data transfer.
 *   
 *   The PCIe driver focuses on the registers for Address Space 0.
 *   
 *   Throughout the LLD, the registers/APIs are grouped into the following:\n\n
 *   -# PCIe Application Registers/APIs\n\n
 *   -# PCIe Configuration Registers/APIs (Local and Remote)\n
 *   2.1  Type0 and Type1 Common Registers/APIs\n
 *   2.2  Type0 Registers/APIs\n  
 *   2.3  Type1 Registers/APIs\n
 *   2.4  MSI Registers/APIs\n
 *   2.5  Capabylity Registers/APIs\n
 *   2.6  Extended Capability Registers/APIs\n
 *   2.7  Port Logic Registers/APIs\n           
 *    
 *   The normal sequence of events to enable the peripheral is listed below.\n
 *   There is a C code example in ti/drv/pcie/example/sample.\n\n
 *
 *   -# Set up the SERDES PLL, reference clock 
 *   -# Set up the peripheral mode (EP/RC)
 *   -# Power up the peripheral 
 *   -# Disable link training 
 *   -# Configure the peripheral, including BAR masks 
 *   -# Configure Inbound Address Translation 
 *   -# Configure Outbound Address Translation
 *   -# Enable Link Training
 *   -# Insure Link Training completion\n
 *   PCIe link is up and ready to be used.
 *
 * In order to check that all values are within bounds, the LLD can be 
 * recompiled with the -Dpcie_DEBUG compiler flag.  This will ensure that 
 * all values passed to writes fit within their assigned bitfields.
 */
 
/* Define pcie LLD Module as a master group in Doxygen format and add all PCIE LLD API 
   definitions to this group. */

/** @defgroup pcielld_module PCIE LLD Module API
 *  @{
 */
/** @} */
   
/** @defgroup pcielld_api_functions PCIE LLD Functions
 *  @ingroup pcielld_module
 */
 
/** @defgroup pcielld_api_macros PCIE LLD Macros
 *  @ingroup pcielld_module
 */

/** @defgroup pcielld_api_structures PCIE LLD API Data Structures
 *  @ingroup pcielld_module
 */

/** @defgroup pcielld_reg_structures PCIE LLD Register Definitions
 *  @ingroup pcielld_module
 */

/** @defgroup pcielld_reg_app_structures PCIE LLD Application Register Definitions
 *  @ingroup pcielld_reg_structures
 */

/** @defgroup pcielld_reg_cfg_structures PCIE LLD Configuration Register Definitions
 *  @ingroup pcielld_reg_structures
 */

/** @defgroup pcielld_reg_cfg_com_structures PCIE LLD Common (Type0/Type1) Register Definitions
 *  @ingroup pcielld_reg_cfg_structures
 */

/** @defgroup pcielld_reg_cfg_type0_structures PCIE LLD Type0 (endpoint) Register Definitions
 *  @ingroup pcielld_reg_cfg_structures
 */

/** @defgroup pcielld_reg_cfg_type1_structures PCIE LLD Type1 (root) Register Definitions
 *  @ingroup pcielld_reg_cfg_structures
 */

/** @defgroup pcielld_reg_cfg_pwr_structures PCIE LLD Power Management Register Definitions
 *  @ingroup pcielld_reg_cfg_structures
 */

/** @defgroup pcielld_reg_cfg_msi_structures PCIE LLD Message Signaled Interrupt Capabilities Register Definitions
 *  @ingroup pcielld_reg_cfg_structures
 */

/** @defgroup pcielld_reg_cfg_cap_structures PCIE LLD Capabilities Register Definitions
 *  @ingroup pcielld_reg_cfg_structures
 */

/** @defgroup pcielld_reg_cfg_cap_ext_structures PCIE LLD Extended Capabilities Register Definitions
 *  @ingroup pcielld_reg_cfg_structures
 */

/** @defgroup pcielld_reg_cfg_pl_structures PCIE LLD Port Logic Register Definitions
 *  @ingroup pcielld_reg_cfg_structures
 */

/** @defgroup pcielld_api_constants PCIE LLD Constants (enum's and define's)
 *  @ingroup pcielld_module
 */

/** These are the possible values for PCIe mode */
typedef enum 
{
  pcie_EP_MODE = 0,    /**< Required when setting the PCIe Mode to End Point using the @ref Pcie_setMode function */
  pcie_LEGACY_EP_MODE, /**< Required when setting the PCIe Mode to Legacy End Point using the @ref Pcie_setMode function */
  pcie_RC_MODE         /**< Required when setting the PCIe Mode to Root Complex using the @ref Pcie_setMode function */
} pcieMode_e;
/* @} */


/**
 *  @ingroup pcielld_api_constants
 *
 *  @{
 */
/** These are the possible values for Prefetch BAR configuration */
typedef enum 
{
  pcie_BAR_NON_PREF = 0,    /**< Non Prefetchable Region*/
  pcie_BAR_PREF             /**< Prefetchable Region*/
} pcieBarPref_e;
/* @} */

/**
 *  @ingroup pcielld_api_constants
 *
 *  @{
 */
/** These are the possible values for Type BAR configuration */
typedef enum 
{
  pcie_BAR_TYPE32 = 0,    /**< 32 bits BAR */
  pcie_BAR_RSVD,          /**< Reserved */
  pcie_BAR_TYPE64         /**< 64 bits BAR */
} pcieBarType_e;
/* @} */

/**
 *  @ingroup pcielld_api_constants
 *
 *  @{
 */
/** These are the possible values for Memory BAR configuration */
typedef enum 
{
  pcie_BAR_MEM_MEM = 0,    /**< Memory BAR */
  pcie_BAR_MEM_IO          /**< IO BAR */
} pcieBarMem_e;
/* @} */

/**
 *  @ingroup pcielld_api_constants
 *
 *  @{
 */
/** These are the possible return values from all PCIE LLD functions */
typedef enum 
{
#ifdef pcie_DEBUG
  /**
   * The call succeeded, but the application could have leaked memory
   * since a non-NULL pointer was overwritten.  This only
   */
  pcie_RET_DBG_WRITE_OVERFLOW = -100L, /**< write value too big for bitfield */
#endif
  pcie_RET_OK = 0,        /**< Call succeeded */
  pcie_RET_INV_HANDLE,    /**< Invalid handle */
  pcie_RET_INV_DEVICENUM, /**< @ref Pcie_open deviceNum invalid */
  pcie_RET_INV_INITCFG,   /**< Invalid Pcie_InitCfg */
  pcie_RET_NO_INIT        /**< Forgot to call Pcie_init() ? */
} pcieRet_e;
/* @} */


/**
 *  @ingroup pcielld_api_constants
 *
 *  @{
 */
/** These are the possible values for the Encoding of LTSSM State in DEBUG0 registers */
typedef enum 
{
  pcie_LTSSM_DETECT_QUIET=0,  
  pcie_LTSSM_DETECT_ACT,  
  pcie_LTSSM_POLL_ACTIVE,  
  pcie_LTSSM_POLL_COMPLIANCE,  
  pcie_LTSSM_POLL_CONFIG,  
  pcie_LTSSM_PRE_DETECT_QUIET,  
  pcie_LTSSM_DETECT_WAIT,  
  pcie_LTSSM_CFG_LINKWD_START,  
  pcie_LTSSM_CFG_LINKWD_ACEPT,  
  pcie_LTSSM_CFG_LANENUM_WAIT,  
  pcie_LTSSM_CFG_LANENUM_ACEPT,  
  pcie_LTSSM_CFG_COMPLETE,  
  pcie_LTSSM_CFG_IDLE,  
  pcie_LTSSM_RCVRY_LOCK,  
  pcie_LTSSM_RCVRY_SPEED,  
  pcie_LTSSM_RCVRY_RCVRCFG,  
  pcie_LTSSM_RCVRY_IDLE,  
  pcie_LTSSM_L0,  
  pcie_LTSSM_L0S,  
  pcie_LTSSM_L123_SEND_EIDLE,  
  pcie_LTSSM_L1_IDLE,  
  pcie_LTSSM_L2_IDLE,  
  pcie_LTSSM_L2_WAKE,  
  pcie_LTSSM_DISABLED_ENTRY,  
  pcie_LTSSM_DISABLED_IDLE,  
  pcie_LTSSM_DISABLED,  
  pcie_LTSSM_LPBK_ENTRY,  
  pcie_LTSSM_LPBK_ACTIVE,  
  pcie_LTSSM_LPBK_EXIT,  
  pcie_LTSSM_LPBK_EXIT_TIMEOUT,  
  pcie_LTSSM_HOT_RESET_ENTRY,  
  pcie_LTSSM_HOT_RESET  
} pcieLtssmState_e;
/* @} */


/**
 *  @ingroup pcielld_api_constants
 *
 *  @{
 */
 /** Selects whether to query or modify the local or remote PCIe registers.\n\n
  *  Important note: PCIe registers are grouped into Application and Configuration registers.\n
  *  This definition of Local/Remote is only applicable to PCIe configuration registers.\n 
  *  It is NOT applicable to PCIe application registers. For application registers, the LLD *always* accesses
  *  LOCAL PCIe application registers.  
  */
typedef enum 
{
  pcie_LOCATION_LOCAL,     /**< Access the local PCIe peripheral */
  pcie_LOCATION_REMOTE     /**< Access the remote PCIe peripheral */
} pcieLocation_e;
/* @} */

/**
 *  @ingroup pcielld_api_constants
 *
 *  @{
 */
/** These are the possible sizes for the PCIe Outbound translation regions */
typedef enum 
{
  pcie_OB_SIZE_1MB = 0,        /**< Corresponds to a region size of 1MB */
  pcie_OB_SIZE_2MB,            /**< Corresponds to a region size of 2MB */
  pcie_OB_SIZE_4MB,            /**< Corresponds to a region size of 4MB */
  pcie_OB_SIZE_8MB             /**< Corresponds to a region size of 8MB */
} pcieObSize_e;
/* @} */

/**
 *  @ingroup pcielld_api_constants
 *
 *  @{
 */
/** These are the Enable/Disable values used by the PCIe Driver */
typedef enum 
{
  pcie_DISABLE = 0,        /**< Disable */
  pcie_ENABLE              /**< Enable  */
} pcieState_e;
/* @} */


/*****************************************************************************
 **********  PCIe APPLICATION REGISTERS  *****************
 ****************************************************************************/

/**
 * @ingroup pcielld_reg_app_structures
 * @brief Specification of the PCIe Peripheral ID Register
 *
 * This Register contains the major and minor revisions 
 * for the PCIe module.
 * 
 * @{
 */
typedef struct pciePidReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /** 
   * @brief [ro] Scheme 
   *
   * Field size: 2 bits
   */
  uint8_t scheme;
  /**
   * @brief [ro] Function code
   *
   * 0xe30 is PCIe
   *
   * Field size: 12 bits
   */
  uint16_t func;
  /**
   * @brief [ro] RTL Version
   *
   * Field size: 5 bits
   */
  uint8_t rtl;
  /**
   * @brief [ro] Major revision
   *
   * Field size: 3 bits
   */
  uint8_t revMaj;
  /**
   * @brief [ro] Customer special version
   *
   * Field size: 2 bits
   */
  uint8_t cust;
  /**
   * @brief [ro] Minor revision
   *
   * Field size: 6 bits
   */
  uint8_t revMin;
} pciePidReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_app_structures
 * @brief Specification of the Command Status Register
 *
 * This Register is used to enable address translation, link training
 * and writing to BAR mask registers.
 *
 * @{
 */
typedef struct pcieCmdStatusReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /** 
   * @brief [rw] Set to enable writing to BAR mask registers that are overlaid on BAR registers. 
   *
   * Field size: 1 bit
   */
  uint8_t dbi;
  /**
   * @brief [rw] Application retry Enable
   *
   * This feature can be used if initialization can take longer than PCIe 
   * stipulated time frame.
   *
   * 1 = Enable all incoming PCIe transactions to be returned with a retry response.
   * 
   * Field size: 1 bit
   */
  uint8_t appRetryEn;
  /**
   * @brief [rw] Posted Write Enable
   *
   * Default is 0 with all internal bus master writes defaulting to non-posted.
   *
   * 1 = Enable the internal bus master to use posted write commands.
   * 
   * Field size: 1 bit
   */
  uint8_t postedWrEn;
  /**
   * @brief [rw] Inbound Translation Enable
   *
   * 1 = Enable translation of inbound memory/IO read/write requests 
   * into memory read/write requests.
   *
   * Field size: 1 bit
   */
  uint8_t ibXltEn;
  /**
   * @brief [rw] Outound Translation Enable
   *
   * 1 = Enable translation of outbound memory read/write requests into 
   * memory/IO/configuration read/write requests.
   *
   * Field size: 1 bit
   */
  uint8_t obXltEn;
  /**
   * @brief [rw] Link Training Enable
   *
   * 1 = Enable LTSSM in PCI Express core and link negotiation with 
   * link partner will begin.
   *
   * Field size: 1 bit
   */
  uint8_t ltssmEn;
} pcieCmdStatusReg_t;
/* @} */


/**
 * @ingroup pcielld_reg_app_structures
 * @brief Specification of the Configuration Transaction Setup Register
 *
 * @{
 */
typedef struct pcieCfgTransReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /** 
   * @brief [rw] Configuration type for outbound configuration accesses 
   *
   * 0 = Type 0 access.
   * 1 = Type 1 access.
   *
   * Field size: 1 bit
   */
  uint8_t type;
  /**
   * @brief [rw] PCIe bus number for outbound configuration accesses
   *
   * Field size: 8 bits
   */
  uint8_t bus;
  /**
   * @brief [rw] PCIe device number for outbound configuration accesses
   *
   * Field size: 5 bit
   */
  uint8_t device;
  /**
   * @brief [rw] PCIe function number for outbound configuration accesses
   *
   * Field size: 3 bits
   */
  uint8_t func;
} pcieCfgTransReg_t;
/* @} */


/**
 * @ingroup pcielld_reg_app_structures
 * @brief Specification of the IO TLP Base Register
 *
 * 
 * @{
 */
typedef struct pcieIoBaseReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] outgoing IO TLP. RC mode only
   *
   * Field size: 20 bits
   *
   */
  uint32_t ioBase;
} pcieIoBaseReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_app_structures
 * @brief Specification of the TLP configuration Register
 *
 * 
 * @{
 */
typedef struct pcieTlpCfgReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /** 
   * @brief [rw] Enable relaxed ordering for all outgoing TLPs 
   *
   * Field size: 1 bit
   */
  uint8_t relaxed;
  /**
   * @brief [rw] Enable No Snoop attribute on all outgoing TLPs
   *
   * Field size: 1 bit
   */
  uint8_t noSnoop;
} pcieTlpCfgReg_t;
/* @} */


/**
 * @ingroup pcielld_reg_app_structures
 * @brief Specification of the Reset Command Register
 *
 * 
 * @{
 */
typedef struct pcieRstCmdReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /** 
   * @brief [ro] Bridge flush status 
   *
   * Used to ensure no pending transactions prior to issuing warm reset.
   * 0 = No transaction is pending.
   * 1 = There are transactions pending.
   * 
   * Field size: 1 bit
   */
  uint8_t flush;
  /**
   * @brief [w1] Write 1 to initiate a downstream hot reset sequence on downstream.
   *
   * Field size: 1 bit
   */
  uint8_t initRst;
} pcieRstCmdReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_app_structures
 * @brief Specification of the Power Management Command Register
 *
 * 
 * @{
 */
typedef struct pciePmCmdReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /** 
   * @brief [w1] PM Turn off 
   *
   * Write 1 to transmit a PM_TURNOFF message. Reads 0. Applicable in RC mode only.
   *
   * 0 = No effect\n
   * 1 = Transmit a PM_TURNOFF message
   * 
   * Field size: 1 bit
   */
  uint8_t turnOff;
  /**
   * @brief [w1] Transmit PM PME message
   *
   * Write 1 to transmit a PM_PME message. Reads 0. Applicable to EP mode only.
   *
   * 0 = No effect\n
   * 1 = Transmit a PM_PME message
   * 
   * Field size: 1 bit
   */
  uint8_t pme;
} pciePmCmdReg_t;
/* @} */


/**
 * @ingroup pcielld_reg_app_structures
 * @brief Specification of the Power Management Configuration Register
 *
 * 
 * @{
 */
typedef struct pciePmCfgReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /** 
   * @brief [rw] PM Turn off 
   *
   * Write 1 to enable entry to L2/L3 ready state. Read to check L2/L3 entry readiness. Applicable to RC and EP.
   *
   * 0 = Disable entry to L2/L3 ready state.\n
   * 1 = Enable entry to L2/L3 ready state.
   * 
   * Field size: 1 bit
   */
  uint8_t entrL23;
} pciePmCfgReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_app_structures
 * @brief Specification of the Activity Status Register
 *
 * 
 * @{
 */
typedef struct pcieActStatusReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /** 
   * @brief [ro] Outbound Buffers Not Empty
   *
   * Field size: 1 bit
   */
  uint8_t obNotEmpty;
  /** 
   * @brief [ro] Inbound Buffers Not Empty
   *
   * Field size: 1 bit
   */
  uint8_t ibNotEmpty;
} pcieActStatusReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_app_structures
 * @brief Specification of the Outbound Size Register
 * 
 * 
 * @{
 */
typedef struct pcieObSizeReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /** 
   * @brief [rw] Set each outbound translation window size 
   *
   * <TABLE>
   * <TR><TH>@ref size</TH><TH>Window Size</TH></TR>
   * <TR><TD>0</TD>        <TD>1 MB</TD></TR>
   * <TR><TD>1</TD>        <TD>2 MB</TD></TR>
   * <TR><TD>2</TD>        <TD>4 MB</TD></TR>
   * <TR><TD>3</TD>        <TD>8 MB</TD></TR>
   * <TR><TD>others</TD>   <TD>reserved</TD></TR>
   * </TABLE>
   *
   * Field size: 3 bits
   */
  uint8_t size;
} pcieObSizeReg_t;
/* @} */

/** 
 * @ingroup pcielld_reg_app_structures 
 * @brief Specification of the Diagnostic Control register
 *
 *
 * @{
 */
typedef struct pcieDiagCtrlReg_s {                            
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */  
  /**
   * @brief [rw] Force ECRC error
   *
   * Write 1 to force inversion of LSB of ECRC for the next one packet. 
   * It is self cleared when the ECRC error has been injected on one TLP.
   *
   * Field size: 1 bit
   */
  uint8_t invEcrc;
  /**
   * @brief [rw] Force LCRC error
   *
   * Write 1 to force inversion of LSB of LCRC for the next one packet. 
   * It is self cleared when the LCRC error has been injected on one TLP.
   *
   * Field size: 1 bit
   */
  uint8_t invLcrc;
} pcieDiagCtrlReg_t;
/* @} */

/** 
 * @ingroup pcielld_reg_app_structures 
 * @brief Specification of the Endian Register 
 *                                                   
 *                                                    
 * @{                                                  
 */                                                     
typedef struct pcieEndianReg_s {                            
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */  
  /** 
   * @brief [rw] Endian mode.               
   *
   * <TABLE>
   * <TR><TH>@ref mode</TH><TH>Endian Swap Mode</TH></TR>
   * <TR><TD>0</TD>        <TD>Swap on 1 byte</TD></TR>
   * <TR><TD>1</TD>        <TD>Swap on 2 bytes</TD></TR>
   * <TR><TD>2</TD>        <TD>Swap on 4 bytes</TD></TR>
   * <TR><TD>3</TD>        <TD>Swap on 8 bytes</TD></TR>
   * </TABLE>
   * 
   * Field size: 2 bits
   */
  uint8_t mode;
} pcieEndianReg_t;
/* @} */

/** 
 * @ingroup pcielld_reg_app_structures 
 * @brief Specification of the Transaction Priority Register 
 *
 *
 * @{
 */
typedef struct pciePriorityReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */  
  /** 
   * @brief [rw] Master PRIV value on master transactions
   *
   * Field size: 1 bits
   */
  uint8_t mstPriv;
  /** 
   * @brief [rw] Master PRIVID value on master transactions
   *
   * Field size: 4 bits
   */
  uint8_t mstPrivID;
  /** 
   * @brief [rw] Priority level for each inbound transaction on the 
   * internal master port
   *
   * Field size: 3 bits
   */
  uint8_t mstPriority;
} pciePriorityReg_t;
/* @} */

/** 
 * @ingroup pcielld_reg_app_structures 
 * @brief Specification of the End of Interrupt Register 
 *
 *
 * @{
 */
typedef struct pcieIrqEOIReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */  
  /** 
   * @brief [wo] EOI for interrupts. 
   *
   * Write to indicate end-of-interrupt for the interrupt events. 
   * Write 0 to mark EOI for INTA, 1 for INTB and so on. 
   * 
   * Field size: 4 bits
   */
  uint8_t EOI;
} pcieIrqEOIReg_t;
/* @} */

/** 
 * @ingroup pcielld_reg_app_structures 
 * @brief Specification of the MSI Interrupt IRQ Register 
 *                                                   
 *                                                    
 * @{                                                  
 */                                                     
typedef struct pcieMsiIrqReg_s {                            
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */  
  /** 
   * @brief [rw] To generate MSI Interrupt 0, the EP should write 0x0000_0000 to this register.                
   *            
   * Field size: 32 bits
   */
  uint32_t msiIrq;
} pcieMsiIrqReg_t;
/* @} */

/** 
 * @ingroup pcielld_reg_app_structures 
 * @brief Specification of the Endpoint Interrupt Request Set Register 
 *                                                   
 *                                                    
 * @{                                                  
 */                                                     
typedef struct pcieEpIrqSetReg_s {                            
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */  
  /** 
   * @brief [rw] Write 1 to generate assert interrupt message                      
   *
   * If MSI is disabled, legacy interrupt assert message will
   * be generated. On read, a 1 indicates currently asserted interrupt.
   *
   * Field size: 1 bit
   */
  uint8_t epIrqSet;
} pcieEpIrqSetReg_t;
/* @} */

/** 
 * @ingroup pcielld_reg_app_structures 
 * @brief Specification of the Endpoint Interrupt Request Clear Register 
 *                                                   
 *                                                    
 * @{                                                  
 */                                                     
typedef struct pcieEpIrqClrReg_s {                            
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */  
  /** 
   * @brief [rw] Write 1 to generate deassert interrupt message.                      
   *
   * If MSI is disabled, legacy interrupt deassert message will be generated. 
   * On read, a 1 indicates currently asserted interrupt.
   * 
   * Field size: 1 bit
   */
  uint8_t epIrqClr;
} pcieEpIrqClrReg_t;
/* @} */

/** 
 * @ingroup pcielld_reg_app_structures 
 * @brief Specification of the Endpoint Interrupt status Register 
 *                                                   
 *                                                    
 * @{                                                  
 */                                                     
typedef struct pcieEpIrqStatusReg_s {                            
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */  
  /** 
   * @brief [rw] Indicates whether interrupt for function 0 is asserted or not                   
   *            
   * Field size: 1 bit
   */
  uint8_t epIrqStatus;
} pcieEpIrqStatusReg_t;
/* @} */

/** 
 * @ingroup pcielld_reg_app_structures 
 * @brief Specification of a General Purpose register
 *
 *
 * @{
 */
typedef struct pcieGenPurposeReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */  
  /** 
   * @brief [rw] Gen Purpose register value
   *
   * Field size: 32 bit
   */
  uint8_t genPurpose;
} pcieGenPurposeReg_t;
/* @} */

/** 
 * @ingroup pcielld_reg_app_structures 
 * @brief Specification of the MSI Raw Interrupt Status Register Register 
 *
 * There are multiple instances (0-7) of this register.
 *
 * @{                                                  
 */                                                     
typedef struct pcieMsiIrqStatusRawReg_s {                            
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */  
  /** 
   * @brief [rw] Each bit indicates raw status of MSI vectors (24, 16, 8, 0) associated with the bit             
   *
   * Typically, writes to this register are only done for debug purposes.
   *
   * Field size: 4 bits
   */
  uint8_t msiRawStatus;
} pcieMsiIrqStatusRawReg_t;
/* @} */


/** 
 * @ingroup pcielld_reg_app_structures 
 * @brief Specification of the MSI Interrupt Enabled Status Register Register 
 *
 * There are multiple instances (0-7) of this register.
 *
 * @{                                                  
 */                                                     
typedef struct pcieMsiIrqStatusReg_s {                            
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */  
  /** 
   * @brief [rw] Each bit indicates status of MSI vector (24, 16, 8, 0) associated with the bit              
   *            
   * Field size: 4 bits
   */
  uint8_t msiIrqStatus;
} pcieMsiIrqStatusReg_t;
/* @} */

/** 
 * @ingroup pcielld_reg_app_structures 
 * @brief Specification of the MSI Interrupt Enable Set Register 
 *
 * There are multiple instances (0-7) of this register.
 *
 * @{                                                  
 */                                                     
typedef struct pcieMsiIrqEnableSetReg_s {                            
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */  
  /** 
   * @brief [rw] Each bit, when written to, enables the MSI interrupt (24, 16, 8, 0) associated with the bit            
   *            
   * Field size: 4 bits
   */
  uint8_t msiIrqEnSet;
} pcieMsiIrqEnableSetReg_t;
/* @} */

/** 
 * @ingroup pcielld_reg_app_structures 
 * @brief Specification of the MSI Interrupt Enable Clear Register 
 *
 * There are multiple instances (0-7) of this register.
 *
 * @{                                                  
 */                                                     
typedef struct pcieMsiIrqEnableClrReg_s {                            
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */  
  /** 
   * @brief [rw] Each bit, when written to, disables the MSI interrupt (24, 16, 8, 0) associated with the bit            
   *            
   * Field size: 4 bits
   */
  uint8_t msiIrqEnClr;
} pcieMsiIrqEnableClrReg_t;
/* @} */

/** 
 * @ingroup pcielld_reg_app_structures 
 * @brief Specification of the Legacy Raw Interrupt Status Register
 *
 * There are multiple instances A-D (0-3) of this register.
 *
 * @{
 */
typedef struct pcieLegacyIrqStatusRawReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */  
  /** 
   * @brief [rw] Legacy Interrupt Raw Status, RC mode only
   *
   * Field size: 1 bit
   */
  uint8_t legacyRawStatus;
} pcieLegacyIrqStatusRawReg_t;
/* @} */


/** 
 * @ingroup pcielld_reg_app_structures 
 * @brief Specification of the Legacy Interrupt Enabled Status Register
 *
 * There are multiple instances (0-7) of this register.
 *
 * @{
 */
typedef struct pcieLegacyIrqStatusReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */  
  /** 
   * @brief [rw] Legacy Interrupt status
   *
   * Set when interrupt is active. Write one to clear the interrupt event.
   * RC mode only.
   *
   * Field size: 1 bits
   */
  uint8_t legacyIrqStatus;
} pcieLegacyIrqStatusReg_t;
/* @} */

/** 
 * @ingroup pcielld_reg_app_structures 
 * @brief Specification of the Legacy Interrupt Enable Set Register 
 *
 * There are multiple instances (0-7) of this register.
 *
 * @{
 */
typedef struct pcieLegacyIrqEnableSetReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */  
  /** 
   * @brief [rw] 0: has no effect; 1: enables the interrupt
   *
   * Field size: 1 bits
   */
  uint8_t legacyIrqEnSet;
} pcieLegacyIrqEnableSetReg_t;
/* @} */

/** 
 * @ingroup pcielld_reg_app_structures 
 * @brief Specification of the Legacy Interrupt Enable Clear Register 
 *
 * There are multiple instances (0-7) of this register.
 *
 * @{
 */
typedef struct pcieLegacyIrqEnableClrReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */  
  /** 
   * @brief [rw] 0 has no effect; 1 disables the interrupt
   *
   * Field size: 1 bits
   */
  uint8_t legacyIrqEnClr;
} pcieLegacyIrqEnableClrReg_t;
/* @} */

/** 
 * @ingroup pcielld_reg_app_structures 
 * @brief Specification of the Raw ERR Interrupt Status Register
 *
 * @{
 */
typedef struct pcieErrIrqStatusRawReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */  
  /** 
   * @brief [rw] ECRC error raw status
   *
   * Field size: 1 bit
   */
  uint8_t errAer;
  /** 
   * @brief [rw] AXI tag lookup fatal error raw status
   *
   * Field size: 1 bit
   */
  uint8_t errAxi;
  /** 
   * @brief [rw] correctable error raw status
   *
   * Field size: 1 bit
   */
  uint8_t errCorr;
  /** 
   * @brief [rw] nonfatal error raw status
   *
   * Field size: 1 bit
   */
  uint8_t errNonFatal;
  /** 
   * @brief [rw] fatal error raw status
   *
   * Field size: 1 bit
   */
  uint8_t errFatal;
  /** 
   * @brief [rw] system error (fatal, nonfatal, correctable error) raw status
   *
   * Field size: 1 bit
   */
  uint8_t errSys;
} pcieErrIrqStatusRawReg_t;
/* @} */

/** 
 * @ingroup pcielld_reg_app_structures 
 * @brief Specification of the ERR Interrupt Enabled Status Register
 *
 * @{
 */
typedef struct pcieErrIrqStatusReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */  
  /** 
   * @brief [rw] ECRC error status
   *
   * Field size: 1 bit
   */
  uint8_t errAer;
  /** 
   * @brief [rw] AXI tag lookup fatal error status
   *
   * Field size: 1 bit
   */
  uint8_t errAxi;
  /** 
   * @brief [rw] correctable error status
   *
   * Field size: 1 bit
   */
  uint8_t errCorr;
  /** 
   * @brief [rw] nonfatal error status
   *
   * Field size: 1 bit
   */
  uint8_t errNonFatal;
  /** 
   * @brief [rw] fatal error status
   *
   * Field size: 1 bit
   */
  uint8_t errFatal;
  /** 
   * @brief [rw] system error (fatal, nonfatal, correctable error) status
   *
   * Field size: 1 bit
   */
  uint8_t errSys;
} pcieErrIrqStatusReg_t;
/* @} */

/** 
 * @ingroup pcielld_reg_app_structures 
 * @brief Specification of the ERR Interrupt Enable Set Register 
 *
 * @{
 */
typedef struct pcieErrIrqEnableSetReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */  
  /** 
   * @brief [rw] set to enable the ECRC error interrupt
   *
   * Field size: 1 bit
   */
  uint8_t errAer;
  /** 
   * @brief [rw] set to enable the AXI tag lookup fatal error interrupt
   *
   * Field size: 1 bit
   */
  uint8_t errAxi;
  /** 
   * @brief [rw] set to enable the correctable error interrupt
   *
   * Field size: 1 bit
   */
  uint8_t errCorr;
  /** 
   * @brief [rw] set to enable the nonfatal error interrupt
   *
   * Field size: 1 bit
   */
  uint8_t errNonFatal;
  /** 
   * @brief [rw] set to enable the fatal error interrupt
   *
   * Field size: 1 bit
   */
  uint8_t errFatal;
  /** 
   * @brief [rw] set to enable the system error (fatal, nonfatal, correctable error) interrupt
   *
   * Field size: 1 bit
   */
  uint8_t errSys;
} pcieErrIrqEnableSetReg_t;
/* @} */

/** 
 * @ingroup pcielld_reg_app_structures 
 * @brief Specification of the ERR Interrupt Enable Clear Register 
 *
 * @{
 */
typedef struct pcieErrIrqEnableClrReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */  
  /** 
   * @brief [rw] set to disable the ECRC error interrupt
   *
   * Field size: 1 bit
   */
  uint8_t errAer;
  /** 
   * @brief [rw] set to disable the AXI tag lookup fatal error interrupt
   *
   * Field size: 1 bit
   */
  uint8_t errAxi;
  /** 
   * @brief [rw] set to disable the correctable error interrupt
   *
   * Field size: 1 bit
   */
  uint8_t errCorr;
  /** 
   * @brief [rw] set to disable the nonfatal error interrupt
   *
   * Field size: 1 bit
   */
  uint8_t errNonFatal;
  /** 
   * @brief [rw] set to disable the fatal error interrupt
   *
   * Field size: 1 bit
   */
  uint8_t errFatal;
  /** 
   * @brief [rw] set to disable the system error (fatal, nonfatal, correctable error) interrupt
   *
   * Field size: 1 bit
   */
  uint8_t errSys;
} pcieErrIrqEnableClrReg_t;
/* @} */

/** 
 * @ingroup pcielld_reg_app_structures 
 * @brief Specification of the Raw Power Management and Reset Interrupt Status Register
 *
 * @{
 */
typedef struct pciePmRstIrqStatusRawReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */  
  /** 
   * @brief [rw] Link Request Reset interrupt raw status
   *
   * Field size: 1 bit
   */
  uint8_t linkRstReq;
  /** 
   * @brief [rw] Power management PME message received interrupt raw status
   *
   * Field size: 1 bit
   */
  uint8_t pmPme;
  /** 
   * @brief [rw] Power mangement ACK received interrupt raw status
   *
   * Field size: 1 bit
   */
  uint8_t pmToAck;
  /** 
   * @brief [rw] Power management turnoff messages received raw status
   *
   * Field size: 1 bit
   */
  uint8_t pmTurnoff;
} pciePmRstIrqStatusRawReg_t;
/* @} */

/** 
 * @ingroup pcielld_reg_app_structures 
 * @brief Specification of the Power Management and Reset Interrupt Enabled Status Register
 *
 * @{
 */
typedef struct pciePmRstIrqStatusReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */  
  /** 
   * @brief [rw] Link Request Reset interrupt status
   *
   * Field size: 1 bit
   */
  uint8_t linkRstReq;
  /** 
   * @brief [rw] Power management PME message received interrupt status
   *
   * Field size: 1 bit
   */
  uint8_t pmPme;
  /** 
   * @brief [rw] Power mangement ACK received interrupt status
   *
   * Field size: 1 bit
   */
  uint8_t pmToAck;
  /** 
   * @brief [rw] Power management turnoff messages received status
   *
   * Field size: 1 bit
   */
  uint8_t pmTurnoff;
} pciePmRstIrqStatusReg_t;
/* @} */

/** 
 * @ingroup pcielld_reg_app_structures 
 * @brief Specification of the Power Management and Reset Interrupt Enable Set Register 
 *
 * @{
 */
typedef struct pciePmRstIrqEnableSetReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */  
  /** 
   * @brief [rw] set to enable the Link Request Reset interrupt
   *
   * Field size: 1 bit
   */
  uint8_t linkRstReq;
  /** 
   * @brief [rw] set to enable the Power management PME message received interrupt
   *
   * Field size: 1 bit
   */
  uint8_t pmPme;
  /** 
   * @brief [rw] set to enable the Power mangement ACK received interrupt 
   *
   * Field size: 1 bit
   */
  uint8_t pmToAck;
  /** 
   * @brief [rw] set to enable the Power management turnoff messages received interrupt
   *
   * Field size: 1 bit
   */
  uint8_t pmTurnoff;
} pciePmRstIrqEnableSetReg_t;
/* @} */

/** 
 * @ingroup pcielld_reg_app_structures 
 * @brief Specification of the Power Management and Reset Interrupt Enable Clear Register 
 *
 * @{
 */
typedef struct pciePmRstIrqEnableClrReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */  
  /** 
   * @brief [rw] set to disable the Link Request Reset interrupt
   *
   * Field size: 1 bit
   */
  uint8_t linkRstReq;
  /** 
   * @brief [rw] set to disable the Power management PME message received interrupt
   *
   * Field size: 1 bit
   */
  uint8_t pmPme;
  /** 
   * @brief [rw] set to disable the Power mangement ACK received interrupt 
   *
   * Field size: 1 bit
   */
  uint8_t pmToAck;
  /** 
   * @brief [rw] set to disable the Power management turnoff messages received interrupt
   *
   * Field size: 1 bit
   */
  uint8_t pmTurnoff;
} pciePmRstIrqEnableClrReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_app_structures
 * @brief Specification of the Outbound Translation Region Offset Low and Index Register
 *
 * There is one register per translation region (0-7)
 * 
 * @{
 */
typedef struct pcieObOffsetLoReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /** 
   * @brief [rw] Offset bits for the translation region
   *
   * Field size: 12 bits
   */
  uint16_t offsetLo;
  /** 
   * @brief [rw] Enable translation region
   *
   * Field size: 1 bit
   */
  uint8_t enable;
} pcieObOffsetLoReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_app_structures
 * @brief Specification of the Outbound Translation Region Offset High Register
 *
 * There is one register per translation region (0-7)
 * 
 * @{
 */
typedef struct pcieObOffsetHiReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /** 
   * @brief [rw] Offset high bits [63:32] for translation region
   *
   * Field size: 32 bits
   */
  uint32_t offsetHi;
} pcieObOffsetHiReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_app_structures
 * @brief Specification of the Inbound Translation BAR Match Register
 *
 * There are multiple instances (0-3) of this register.
 *
 * @{
 */
typedef struct pcieIbBarReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /** 
   * @brief [rw] BAR number to match for inbound translation region   
   *
   * Field size: 3 bits
   */
  uint8_t ibBar;
} pcieIbBarReg_t;
/* @} */


/**
 * @ingroup pcielld_reg_app_structures
 * @brief Specification of the Inbound Translation Start Address Low Register
 *
 * There are multiple instances (0-3) of this register.
 *
 * @{
 */
typedef struct pcieIbStartLoReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /** 
   * @brief [rw] Start address bits [31:8] for inbound translation region
   *
   * Field size: 24 bits
   */
  uint32_t ibStartLo;
} pcieIbStartLoReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_app_structures
 * @brief Specification of the Inbound Translation Start Address High Register
 *
 * There are multiple instances (0-3) of this register.
 *
 * @{
 */
typedef struct pcieIbStartHiReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /** 
   * @brief [rw] Start address high bits [63:32] for inbound translation region 
   *
   * Field size: 32 bits
   */
  uint32_t ibStartHi;
} pcieIbStartHiReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_app_structures
 * @brief Specification of the Inbound Translation Address Offset Register
 *
 * There are multiple instances (0-3) of this register.
 *
 * @{
 */
typedef struct pcieIbOffsetReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /** 
   * @brief [rw] Offset address bits [31:8] for inbound translation region
   *
   * Field size: 24 bits
   */
  uint32_t ibOffset;
} pcieIbOffsetReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_app_structures
 * @brief Specification of the PCS Configuration 0 Register
 *
 * @{
 */
typedef struct pciePcsCfg0Reg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /** 
   * @brief [rw] Receiver lock/sync control.
   *
   * Field size: 5 bits
   */
  uint8_t pcsSync;
  /** 
   * @brief [rw] Receiver initialization holdoff control. 
   *
   * Field size: 8 bits
   */
  uint8_t pcsHoldOff;
  /** 
   * @brief [rw] Rate change delay.
   *
   * Field size: 2 bits
   */
  uint8_t pcsRCDelay;
  /** 
   * @brief [rw] Detection delay.
   *
   * Field size: 4 bits
   */
  uint8_t pcsDetDelay;
  /** 
   * @brief [rw] Enable short times for debug purposes.
   *
   * Field size: 1 bits
   */
  uint8_t pcsShrtTM;
  /** 
   * @brief [rw] Enable PIPE Spec 1.86 for phystatus behavior.
   *
   * Field size: 1 bits
   */
  uint8_t pcsStat186;
  /** 
   * @brief [rw] Fed term output to 3'b100 during reset.
   *
   * Field size: 1 bits
   */
  uint8_t pcsFixTerm;
  /** 
   * @brief [rw] Fix std output to 2'b10.
   *
   * Field size: 1 bits
   */
  uint8_t pcsFixStd;
  /** 
   * @brief [rw] Deassert enidl during L2 state.
   *
   * Field size: 1 bits
   */
  uint8_t pcsL2EnidlOff;
  /** 
   * @brief [rw] Deassert Rx enable in L0s state.
   *
   * Field size: 1 bits
   */
  uint8_t pcsL2L0SRxOff;
  /** 
   * @brief [rw] RX and TX on during reset and TX also on in P1 state.
   *
   * Field size: 1 bits
   */
  uint8_t pcsRxTxOn;
  /** 
   * @brief [rw] RX and TX on during reset.
   *
   * Field size: 1 bits
   */
  uint8_t pcsRxTxRst;
} pciePcsCfg0Reg_t;
/* @} */

/**
 * @ingroup pcielld_reg_app_structures
 * @brief Specification of the PCS Configuration 1 Register
 *
 * @{
 */
typedef struct pciePcsCfg1Reg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /** 
   * @brief [rw] Error bit enable. 
   *
   * Field size: 10 bits
   */
  uint16_t pcsErrBit;
  /** 
   * @brief [rw] Error lane enable
   *
   * Field size: 2 bits
   */
  uint8_t pcsErrLn;
  /** 
   * @brief [rw] Error injection mode
   *
   * Field size: 2 bits
   */
  uint8_t pcsErrMode;
} pciePcsCfg1Reg_t;
/* @} */

/**
 * @ingroup pcielld_reg_app_structures
 * @brief Specification of the PCS Status Register
 *
 * @{
 */
typedef struct pciePcsStatusReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /** 
   * @brief [ro] PCS RTL Revision.
   *
   * Field size: 3 bits
   */
  uint8_t pcsRev;
  /** 
   * @brief [ro] PCS lanes enabled status.
   *
   * Field size: 2 bits
   */
  uint8_t pcsLnEn;
  /** 
   * @brief [ro] PCS transmitters enabled status.
   *
   * Field size: 2 bits
   */
  uint8_t pcsTxEn;
  /** 
   * @brief [ro] PCS receivers enabled status.
   *
   * Field size: 2 bits
   */
  uint8_t pcsRxEn;
} pciePcsStatusReg_t;
/* @} */

/** 
 * @ingroup pcielld_reg_app_structures 
 * @brief Specification of the SERDES config 0 Register 
 *                                                   
 *                                                    
 * @{                                                  
 */                                                     
typedef struct pcieSerdesCfg0Reg_s {                            
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */  
  /** 
   * @brief [rw] Enable Tx loopback. Set both bits high to enable.        
   *            
   * Field size: 2 bits
   */
  uint8_t txLoopback;
  /** 
   * @brief [rw] Master mode for synchronization.             
   *            
   * Field size: 1 bit
   */
  uint8_t txMsync;
  /** 
   * @brief [rw] Enable common mode adjustment.             
   *            
   * Field size: 1 bit
   */
  uint8_t txCm;
  /** 
   * @brief [rw] Invert Tx pair polarity.             
   *            
   * Field size: 1 bit
   */
  uint8_t txInvpair;
  /** 
   * @brief [rw] Enable Rx loopback. Set both bits to high to enable loopback.      
   *            
   * Field size: 2 bits
   */
  uint8_t rxLoopback;
  /** 
   * @brief [rw] Enable Rx offset compensation.             
   *            
   * Field size: 1 bit
   */
  uint8_t rxEnoc;
  /** 
   * @brief [rw] Enable Rx adaptive equalization.             
   *            
   * Field size: 4 bits
   */
  uint8_t rxEq;
  /** 
   * @brief [rw] Enable Rx clock data recovery.            
   *            
   * Field size: 3 bits
   */
  uint8_t rxCdr;
  /** 
   * @brief [rw] Enable Rx loss of signal detection.           
   *            
   * Field size: 3 bits
   */
  uint8_t rxLos;
  /** 
   * @brief [rw] Enable Rx symbol alignment.             
   *            
   * Field size: 2 bits
   */
  uint8_t rxAlign;
  /** 
   * @brief [rw] Invert Rx pair polarity.             
   *            
   * Field size: 1 bit
   */
  uint8_t rxInvpair;
} pcieSerdesCfg0Reg_t;
/* @} */

/** 
 * @ingroup pcielld_reg_app_structures 
 * @brief Specification of the SERDES config 1 Register 
 *                                                   
 *                                                    
 * @{                                                  
 */                                                     
typedef struct pcieSerdesCfg1Reg_s {                            
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */  
  /** 
   * @brief [rw] Enable Tx loopback. Set both bits high to enable.        
   *            
   * Field size: 2 bits
   */
  uint8_t txLoopback;
  /** 
   * @brief [rw] Master mode for synchronization.             
   *            
   * Field size: 1 bit
   */
  uint8_t txMsync;
  /** 
   * @brief [rw] Enable common mode adjustment.             
   *            
   * Field size: 1 bit
   */
  uint8_t txCm;
  /** 
   * @brief [rw] Invert Tx pair polarity.             
   *            
   * Field size: 1 bit
   */
  uint8_t txInvpair;
  /** 
   * @brief [rw] Enable Rx loopback. Set both bits to high to enable loopback.      
   *            
   * Field size: 2 bits
   */
  uint8_t rxLoopback;
  /** 
   * @brief [rw] Enable Rx offset compensation.             
   *            
   * Field size: 1 bit
   */
  uint8_t rxEnoc;
  /** 
   * @brief [rw] Enable Rx adaptive equalization.             
   *            
   * Field size: 4 bits
   */
  uint8_t rxEq;
  /** 
   * @brief [rw] Enable Rx clock data recovery.            
   *            
   * Field size: 3 bits
   */
  uint8_t rxCdr;
  /** 
   * @brief [rw] Enable Rx loss of signal detection.           
   *            
   * Field size: 3 bits
   */
  uint8_t rxLos;
  /** 
   * @brief [rw] Enable Rx symbol alignment.             
   *            
   * Field size: 2 bits
   */
  uint8_t rxAlign;
  /** 
   * @brief [rw] Invert Rx pair polarity.             
   *            
   * Field size: 1 bit
   */
  uint8_t rxInvpair;
} pcieSerdesCfg1Reg_t;
/* @} */



/*****************************************************************************
 **********  PCIe LOCAL/REMOTE CONFIG TYPE 0 and TYPE 1 REGISTERS ************
 **********        Registers that are common to both Types        ************
 ****************************************************************************/

/**
 * @ingroup pcielld_reg_cfg_com_structures
 * @brief Specification of the Vendor Device ID Register
 *
 * 
 * @{
 */
typedef struct pcieVndDevIdReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /** 
   * @brief [rw] Device ID
   *
   * Field size: 16 bits
   */
  uint16_t devId;
  /** 
   * @brief [rw] Vendor ID
   *
   * Field size: 16 bits
   */
  uint16_t vndId;
} pcieVndDevIdReg_t;
/* @} */


/**
 * @ingroup pcielld_reg_cfg_com_structures
 * @brief Specification of the Status Command Register
 *
 * @{
 */
typedef struct pcieStatusCmdReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /** 
   * @brief [rw] parity 
   *
   * Set if received a poisoned TLP
   *
   * Field size: 1 bit
   */
  uint8_t parity;
  /**
   * @brief [rw] sys error
   *
   * Set if function sends an ERR_FATAL or ERR_NONFATAL message and 
   * @ref serrEn bit is set
   *
   * Field size: 1 bit
   */
  uint8_t sysError;
  /**
   * @brief [rw] mst abort
   *
   * Set when a requester receives a completion with unsupported request 
   * completion status
   *
   * Field size: 1 bit
   */
  uint8_t mstAbort;
  /**
   * @brief [rw] tgt abort
   *
   * Set when a requester receives a completion with completer abort status.
   *
   * Field size: 1 bit
   */
  uint8_t tgtAbort;
  /**
   * @brief [rw] sig tgt abort
   *
   * Set when a function acting as a completer terminates a request by issuing 
   * completer abort completion status to the requester.
   * 
   * Field size: 1 bit
   */
  uint8_t sigTgtAbort;
  /**
   * @brief [rw] par error
   *
   * This bit is set by a requester if the @ref parError bit is set 
   * in its Command register and either the condition that the requester 
   * receives a poisoned completion or the condition that the
   * requester poisons a write request is true.
   * 
   * Field size: 1 bit
   */
  uint8_t parError;
  /**
   * @brief [ro] cap list
   *
   * For PCIe, this field must be set to 1.
   *
   * Field size: 1 bit
   */
  uint8_t capList;
  /**
   * @brief [rw] stat
   *
   * Indicates that the function has received an interrupt.
   *
   * Field size: 1 bit
   */
  uint8_t stat;
  /**
   * @brief [ro] dis
   *
   * Setting this bit disables generation of INTx messages.
   *
   * Field size: 1 bit
   */
  uint8_t dis;
  /**
   * @brief [rw] serr en
   *
   * When set, it enables generation of the appropriate PCI Express error 
   * messages to the Root Complex.
   *
   * Field size: 1 bit
   */
  uint8_t serrEn;
  /**
   * @brief [rw] resp
   *
   * This bit controls whether or not the device responds to detected 
   * parity errors (poisoned TLP). This error is typically reported as an 
   * unsupported request and may also result in a non-fatal error
   * message if @ref serrEn = 1. If this bit is set, the PCIESS will respond 
   * normally to parity errors. If this bit is cleared, the PCIESS 
   * will ignore detected parity errors.
   * 
   * Field size: 1 bit
   */
  uint8_t resp;
  /**
   * @brief [rw] enables mastership of the bus
   *
   * Field size: 1 bit
   */
  uint8_t busMs;
  /**
   * @brief [rw] enables device to respond to memory access
   *
   * Field size: 1 bit
   */
  uint8_t memSp;
  /**
   * @brief [rw] enables device to respond to IO access
   *
   * This functionality is not supported in PCIESS and therefore 
   * this bit is set to 0.
   * 
   * Field size: 1 bit
   */
  uint8_t ioSp;
} pcieStatusCmdReg_t;
/* @} */


/**
 * @ingroup pcielld_reg_cfg_com_structures
 * @brief Specification of the Class code and revision ID Register
 *
 * 
 * @{
 */
typedef struct pcieRevIdReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /** 
   * @brief [ro] Class Code
   *
   * Field size: 24 bits
   */
  uint32_t classCode;
  /** 
   * @brief [ro] Revision ID
   *
   * Field size: 8 bits
   */
  uint8_t revId;
} pcieRevIdReg_t;
/* @} */


/**
 * @ingroup pcielld_reg_cfg_type0_structures
 * @brief Specification of the Base Address Register (BAR)
 *
 * This should be used to access a BAR register.
 *
 * There are two situations when this structure should be used:\n
 * 1. When setting up a 32 bit BAR\n
 * 2. When setting up the lower 32bits of a 64bits BAR 
 *
 * Refer to @ref pcieBar32bitReg_t for the other possible BAR configurations
 *
 * @{
 */
typedef struct pcieBarReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /** 
   * @brief [rw] Base Address 
   *
   * Field size: 28 bits
   */
  uint32_t base;
  /**
   * @brief [rw] Prefetchable region?
   *
   * For memory BARs, it indicates whether the region is prefetchable.\n
   * 0 = Non-prefetchable.\n
   * 1 = Prefetchable.
   *
   * For I/O Bars, it is used as second least significant bit (LSB) 
   * of the base address.
   * 
   * Field size: 1 bit
   */
  uint8_t prefetch;
  /**
   * @brief [rw] Bar Type 
   *
   * For memory BARs, they determine the BAR type.\n
   * 0h = 32-bit BAR.\n
   * 2h = 64-bit BAR.\n
   * Others = Reserved.
   *
   * For I/O BARs, bit 2 is the least significant bit (LSB) of the 
   * base address and bit 1 is 0.
   * 
   * Field size: 2 bits
   */
  uint8_t type;
  /**
   * @brief [rw] Memory or IO BAR
   *
   * 0 = Memory BAR.\n
   * 1 = I/O BAR.
   * 
   * Field size: 1 bit
   */
  uint8_t memSpace;
} pcieBarReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_type0_structures
 * @brief Specification of the Base Address Register (BAR).
 *
 * This should be used to read/write a 32bit word to the BAR register.
 *
 * There are two situations when this structure should be used:\n
 * 1. When setting up BAR masks\n
 * 2. When setting up the upper 32bits of a 64bits BAR 
 *
 * Refer to @ref pcieBarReg_t for the other possible BAR configurations
 *
 * @{
 */
typedef struct pcieBar32bitReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /** 
   * @brief [rw] 32bits word (BAR mask or BAR address) 
   *
   * Field size: 32 bits
   */
  uint32_t reg32;
} pcieBar32bitReg_t;
/* @} */

/*****************************************************************************
 **********  PCIe LOCAL/REMOTE CONFIG TYPE 0 REGISTERS  **********************
 ****************************************************************************/

/**
 * @ingroup pcielld_reg_cfg_type0_structures
 * @brief Specification of the BIST Header Register
 *
 * 
 * @{
 */
typedef struct pcieBistReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /** 
   * @brief [ro] Bist capability
   *
   * Returns a one for BIST capability and zero otherwise. Not supported by PCIESS.
   *
   * Field size: 1 bit
   */
  uint8_t bistCap;
  /** 
   * @brief [ro] Start Bist
   *
   * Write a one to start BIST. Not supported by PCIESS.
   *
   * Field size: 1 bit
   */
  uint8_t startBist;
  /** 
   * @brief [ro]  Completion code
   *
   * Not supported by PCIESS.
   *
   * Field size: 4 bits
   */
  uint8_t compCode;
  /** 
   * @brief [ro]  Multifunction device
   *
   * Field size:  1 bit
   */
  uint8_t mulfunDev;
  /** 
   * @brief [ro]  Header type
   *
   * Configuration header format.
   *
   * 0 = EP mode\n
   * 1 = RC mode
   * 
   * Field size: 7 bits
   */
  uint8_t hdrType;
  /** 
   * @brief [ro] Not applicable in PCIe
   *
   * Field size:  8 bits
   */
  uint8_t latTmr;
  /** 
   * @brief [ro] Not applicable in PCIe 
   *
   * Field size:  8 bits
   */
  uint8_t cacheLnSize;
} pcieBistReg_t;
/* @} */


/**
 * @ingroup pcielld_reg_cfg_type0_structures
 * @brief @ref pcieBarReg_s register plus an index (End Point BAR)
 *
 * There are multiple instances of this register. The index is used to select which
 * instance of the register will be accessed.
 *
 * This structure is used to access an End Point BAR. For more details, please refer to @ref pcieBarReg_t
 *
 * @{
 */
typedef struct pcieType0BarIdx_s {
  pcieBarReg_t reg;  /**< @brief Register Structure */
  uint8_t      idx;  /**< @brief Index in the array of registers of this type */
} pcieType0BarIdx_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_type0_structures
 * @brief @ref pcieBar32bitReg_s register plus an index (End Point BAR)
 *
 * There are multiple instances of this register. The index is used to select which
 * instance of the register will be accessed.
 *
 * This structure is used to access an End Point BAR. For more details, please refer to @ref pcieBar32bitReg_t
 *
 * @{
 */
typedef struct pcieType0Bar32bitIdx_s {
  pcieBar32bitReg_t reg;  /**< @brief Register Structure */
  uint8_t           idx;  /**< @brief Index in the array of registers of this type */
} pcieType0Bar32bitIdx_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_type0_structures
 * @brief Specification of the Subsystem Vendor ID Register
 *
 * 
 * @{
 */
typedef struct pcieSubIdReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /** 
   * @brief [ro] Subsystem ID
   *
   * Field size: 16 bits
   */
  uint16_t subId;
  /** 
   * @brief [ro] Subsystem Vendor ID
   *
   * Field size: 16 bits
   */
  uint16_t subVndId;
} pcieSubIdReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_type0_structures
 * @brief Specification of the Expansion ROM Register
 *
 * 
 * @{
 */
typedef struct pcieExpRomReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /** 
   * @brief [ro] Address of Expansion ROM
   *
   * Field size: 21 bits
   */
  uint32_t expRomAddr;
  /** 
   * @brief [ro] Expansion ROM Enable
   *
   * Field size: 1 bit
   */
  uint8_t enable;
} pcieExpRomReg_t;
/* @} */


/**
 * @ingroup pcielld_reg_cfg_type0_structures
 * @brief Specification of the Capability Pointer Register
 *
 * 
 * @{
 */
typedef struct pcieCapPtrReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /** 
   * @brief [rw] First Capability Pointer
   *
   * By default, it points to Power Management Capability structure.
   * 
   * Field size: 8 bits
   */
  uint8_t ptr;
} pcieCapPtrReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_type0_structures
 * @brief Specification of the Interrupt Pin Register
 *
 * 
 * @{
 */
typedef struct pcieIntPinReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /** 
   * @brief [ro] interrupt Pin
   *
   * It identifies the legacy interrupt message that the device uses. 
   * For single function configuration, the core only uses INTA. 
   *
   * <TABLE>
   * <TR><TH>@ref intPin</TH><TH>Legacy Interrupt</TH></TR>
   * <TR><TD>0</TD>          <TD>none</TD></TR>
   * <TR><TD>1</TD>          <TD>INTA</TD></TR>
   * <TR><TD>2</TD>          <TD>INTB</TD></TR>
   * <TR><TD>3</TD>          <TD>INTC</TD></TR>
   * <TR><TD>4</TD>          <TD>INTD</TD></TR>
   * <TR><TD>others</TD>     <TD>reserved</TD></TR>
   * </TABLE>
   * 
   * Field size: 8 bits
   */
  uint8_t intPin;
  /** 
   * @brief [rw] interrupt line 
   *
   * Field size: 8 bits
   */
  uint8_t intLine;
} pcieIntPinReg_t;
/* @} */


/*****************************************************************************
 **********  PCIe LOCAL/REMOTE CONFIG TYPE 1 REGISTERS  **********************
 ****************************************************************************/

/**
 * @ingroup pcielld_reg_cfg_type1_structures
 * @brief @ref pcieBarReg_s register plus an index (Root Complex BAR)
 *
 * There are multiple instances of this register. The index is used to select which
 * instance of the register will be accessed.
 *
 * This structure is used to access a Root Complex BAR. For more details, please refer to @ref pcieBarReg_t.
 *
 * @{
 */
typedef struct pcieType1BarIdx_s {
  pcieBarReg_t reg;  /**< @brief Register Structure */
  uint8_t      idx;  /**< @brief Index in the array of registers of this type */
} pcieType1BarIdx_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_type1_structures
 * @brief @ref pcieBar32bitReg_s register plus an index (Root Complex BAR)
 *
 * There are multiple instances of this register. The index is used to select which
 * instance of the register will be accessed.
 *
 * This structure is used to access a Root Complex BAR. For more details, please refer to @ref pcieBar32bitReg_t.
 *
 * @{
 */
typedef struct pcieType1Bar32bitIdx_s {
  pcieBar32bitReg_t reg;  /**< @brief Register Structure */
  uint8_t           idx;  /**< @brief Index in the array of registers of this type */
} pcieType1Bar32bitIdx_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_type1_structures
 * @brief Specification of the BIST, Header Type, Latency Time and Cache Line Size Regiser
 *
 * @{
 */
typedef struct pcieType1BistHeaderReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /** 
   * @brief [ro] Returns a 1 for BIST capability and 0 otherwise. 
   *
   * Not supported by PCIESS.
   *
   * Field size: 1 bit
   */
  uint8_t bistCap;
  /** 
   * @brief [ro] Write a one to start BIST. 
   *
   * Not supported by PCIESS.
   *
   * Field size: 1 bit
   */
  uint8_t startBist;
  /** 
   * @brief [rw] Completion Code. 
   *
   * Not supported by PCIESS.
   *
   * Field size: 4 bits
   */
  uint8_t compCode;
  /** 
   * @brief [rw] Returns 1 if it is a multi-function device.
   *
   * Field size: 1 bit
   */
  uint8_t mulFunDev;
  /** 
   * @brief [rw] Configuration Header Format.
   *
   * 0 = EP mode\n
   * 1 = RC mode
   *
   * Field size: 7 bits
   */
  uint8_t hdrType;
  /** 
   * @brief [ro] Not applicable in PCIe
   *
   * Field size: 8 bits
   */
  uint8_t latTmr;
  /** 
   * @brief [ro] Not applicable in PCIe
   *
   * Field size: 8 bits
   */
  uint8_t cacheLnSize;
} pcieType1BistHeaderReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_type1_structures
 * @brief Specification of the Latency Timer and Bus Number Register
 *
 * @{
 */
typedef struct pcieType1BusNumReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /** 
   * @brief [ro] Secondary Latency Timer (N/A for PCIe)
   *
   * Field size: 8 bits
   */
  uint8_t secLatTmr;
  /** 
   * @brief [rw] Subordinate Bus Number. This is highest bus 
   *             number on downstream interface.
   *
   * Field size: 8 bits
   */
  uint8_t subBusNum;
  /** 
   * @brief [rw] Secondary Bus Number. It is typically 1h for RC.
   *
   * Field size: 8 bits
   */
  uint8_t secBusNum;
  /** 
   * @brief [rw] Primary Bus Number. It is 0 for RC and nonzero for 
   *             switch devices only.
   *
   * Field size: 8 bits
   */
  uint8_t priBusNum;
} pcieType1BusNumReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_type1_structures
 * @brief Specification of the Secondary Status and IO Base/Limit Register
 *
 * @{
 */
typedef struct pcieType1SecStatReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /** 
   * @brief [rw] Detected Parity Error.  
   *
   * Read 1 if received a poisoned TLP.
   * Write 1 to clear; write 0 has no effect.
   *
   * Field size: 1 bits
   */
  uint8_t dtctPError;
  /** 
   * @brief [rw] Received System Error. 
   *
   * Read 1 if received an ERR_FATAL or ERR_NONFATAL message.
   * Write 1 to clear; write 0 has no effect.
   *
   * Field size: 1 bits
   */
  uint8_t rxSysError;
  /** 
   * @brief [rw] Received Master Abort. 
   *
   * Read 1 if received a completion with unsupported request completion status.
   * Write 1 to clear; write 0 has no effect.
   *
   * Field size: 1 bits
   */
  uint8_t rxMstAbort;
  /** 
   * @brief [rw] Received Target Abort. 
   *
   * Read 1 if received a completion with completer abort completion status.
   * Write 1 to clear; write 0 has no effect.
   *
   * Field size: 1 bits
   */
  uint8_t rxTgtAbort;
  /** 
   * @brief [rw] Signaled Target Abort. 
   *
   * Read 1 if sent a posted or non-posted request as a completer abort error.
   * Write 1 to clear; write 0 has no effect.
   *
   * Field size: 1 bits
   */
  uint8_t txTgtAbort;
  /** 
   * @brief [rw] Master Data Parity Error. 
   *
   * Read 1 if the parity error enable bit 
   * @ref pcieType1BridgeIntReg_s::pErrRespEn is set and either the condition 
   * that the requester receives a poisoned completion or the condition 
   * that the requester poisons a write request is true.
   * Write 1 to clear; write 0 has no effect.
   *
   * Field size: 1 bits
   */
  uint8_t mstDPErr;
  /** 
   * @brief [rw] Upper 4 bits of 16bit IO Space Limit Address.
   *
   * Field size: 4 bits
   */
  uint8_t IOLimit;
  /** 
   * @brief [rw] Indicates addressing for IO Limit Address. 
   *
   * Writable from internal bus interface.
   * 0 = 16-bit IO addressing.
   * 1 = 32-bit IO addressing.
   *
   * Field size: 1 bits
   */
  uint8_t IOLimitAddr;
  /** 
   * @brief [rw] Upper 4 bits of 16bit IO Space Base Address.
   *
   * Field size: 4 bits
   */
  uint8_t IOBase;
  /** 
   * @brief [rw] Indicates addressing for the IO Base Address. 
   *
   * Writable from internal bus interface.
   * 0 = 16-bit IO addressing.
   * 1 = 32-bit IO addressing.
   * 
   * Field size: 1 bits
   */
  uint8_t IOBaseAddr;
} pcieType1SecStatReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_type1_structures
 * @brief Specification of the Memory Limit and Base Register
 *
 * @{
 */
typedef struct pcieType1MemspaceReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /** 
   * @brief [rw] Upper 12 bits of 32bit Memory Limit Address.
   *
   * Field size: 12 bits
   */
  uint16_t limit;
  /**
   * @brief [rw] Upper 12 bits of 32bit Memory Base Address.
   *
   * Field size: 12 bit
   */
  uint16_t base;
} pcieType1MemspaceReg_t;
/* @} */


/**
 * @ingroup pcielld_reg_cfg_type1_structures
 * @brief Specification of the Prefetchable Memory Limit and Base Register
 *
 * @{
 */
typedef struct pciePrefMemReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /** 
   * @brief [rw] Memory limit 
   *
   * Upper 12 bits of 32bit prefetchable memory limit address (end address).
   *
   * Field size: 12 bits
   */
  uint16_t limit;
  /**
   * @brief [rw] 32 or 64 bit addressing
   *
   * Indicates addressing for prefetchable memory limit address (end address).
   *
   * 0 = 32-bit memory addressing\n
   * 1 = 64-bit memory addressing
   * 
   * Field size: 1 bit
   */
  uint8_t limitAddr;
  /** 
   * @brief [rw] Memory base 
   *
   * Upper 12 bits of 32bit prefetchable memory base address (start address).
   *
   * Field size: 12 bits
   */
  uint16_t base;
  /**
   * @brief [rw] 32 or 64 bit addressing
   *
   * Indicates addressing for the prefetchable memory base address (start address).
   *
   * 0 = 32-bit memory addressing\n
   * 1 = 64-bit memory addressing
   *
   * Field size: 1 bit
   */
  uint8_t baseAddr;
} pciePrefMemReg_t;
/* @} */


/**
 * @ingroup pcielld_reg_cfg_type1_structures
 * @brief Specification of the Prefetchable Memory Base Upper Register
 *
 * 
 * @{
 */
typedef struct pciePrefBaseUpperReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Base upper 32bits 
   *
   * Upper 32 bits of Prefetchable Memory Base Address. Used with 64bit 
   * prefetchable memory addressing only.
   * 
   * Field size: 32 bits
   *
   */
  uint32_t base;
} pciePrefBaseUpperReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_type1_structures
 * @brief Specification of the Prefetchable Memory Limit Upper Register
 *
 * 
 * @{
 */
typedef struct pciePrefLimitUpperReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Base upper 32bits 
   *
   * Upper 32 bits of Prefetchable Memory Limit Address. Used with 64 bit 
   * prefetchable memory addressing only.
   * 
   * Field size: 32 bits
   *
   */
  uint32_t limit;
} pciePrefLimitUpperReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_type1_structures
 * @brief Specification of the IO Base and Limit Upper 16 bits Register
 *
 * @{
 */
typedef struct pcieType1IOSpaceReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Upper 16 bits of IO Base Address. 
   *
   * Used with 32 bit IO space addressing only.
   *
   * Field size: 16 bits
   *
   */
  uint16_t IOBase;
  /**
   * @brief [rw] Upper 16 bits of IO Limit Address. 
   *
   * Used with 32 bit IO space addressing only. 
   *
   * Field size: 16 bits
   *
   */
  uint16_t IOLimit;
} pcieType1IOSpaceReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_type1_structures
 * @brief Specification of the Capabilities Pointer Register
 *
 * @{
 */
typedef struct pcieType1CapPtrReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] First Capability Pointer. 
   *
   * By default, it points to Power Management Capability structure.
   * Writable from internal bus interface.
   *
   * Field size: 8 bits
   *
   */
  uint8_t capPtr;
} pcieType1CapPtrReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_type1_structures
 * @brief Specification of the Expansion ROM Base Address Register
 * 
 * @{
 */
typedef struct pcieType1ExpnsnRomReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Address of Expansion ROM
   *
   * Field size: 21 bits [0-0x1FFFFF]
   *
   */
  uint32_t expRomBaseAddr;
  /**
   * @brief [rw] Expansion ROM enable
   *
   * Field size: 1 bit
   *
   */
  uint8_t expRomEn;

} pcieType1ExpnsnRomReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_type1_structures
 * @brief Specification of the Bridge Control and Interrupt Register
 * 
 * @{
 */
typedef struct pcieType1BridgeIntReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [ro] Discard Timer SERR Enable Status. 
   *
   * Not Applicable to PCI Express. Hardwired to 0. 
   *
   * Field size: 1 bit
   *
   */
  uint8_t serrEnStatus;
  /**
   * @brief [ro] Discard Timer Status. 
   *
   * Not applicable to PCI Express. Hardwired to 0.
   *
   * Field size: 1 bit
   *
   */
  uint8_t timerStatus;
  /**
   * @brief [ro] Secondary Discard Timer.
   *
   * Not applicable to PCI Express. Hardwired to 0.
   *
   * Field size: 1 bit
   *
   */
  uint8_t secTimer;
  /**
   * @brief [ro] Primary Discard Timer.
   *
   * Not applicable to PCI Express. Hardwired to 0.
   *
   * Field size: 1 bit
   *
   */
  uint8_t priTimer;
  /**
   * @brief [ro] Fast Back to Back Transactions Enable.
   *
   * Not applicable to PCI Express. Hardwired to 0.
   *
   * Field size: 1 bit
   *
   */
  uint8_t b2bEn;
  /**
   * @brief [rw] Secondary Bus Reset.
   *
   * Field size: 1 bit
   *
   */
  uint8_t secBusRst;
  /**
   * @brief [ro] Master Abort Mode. 
   *
   * Not applicable to PCI Express. Hardwired to 0.
   *
   * Field size: 1 bit
   *
   */
  uint8_t mstAbortMode;
  /**
   * @brief [rw] VGA 16 bit Decode
   *
   * Field size: 1 bit
   *
   */
  uint8_t vgaDecode;
  /**
   * @brief [rw] VGA Enable
   *
   * Field size: 1 bit
   *
   */
  uint8_t vgaEn;
  /**
   * @brief [rw] ISA Enable
   *
   * Field size: 1 bit
   *
   */
  uint8_t isaEn;
  /**
   * @brief [rw] SERR Enable. 
   *
   * Set to enable forwarding of ERR_COR, ERR_NONFATAL and ERR_FATAL messages.
   * 
   * Field size: 1 bit
   *
   */
  uint8_t serrEn;
  /**
   * @brief [rw] Parity Error Response Enable.
   *
   * This bit controls the logging of poisoned TLPs in 
   * @ref pcieType1SecStatReg_s::mstDPErr
   * 
   * Field size: 1 bit
   *
   */
  uint8_t pErrRespEn;
  /**
   * @brief [rw] Interrupt Pin. 
   *
   * It identifies the legacy interrupt message that the device uses. 
   * For single function configuration, the core only uses INTA. This register 
   * is writable through internal bus interface.
   *
   *  0  = Legacy interrupt is not being used
   *  1h = INTA
   *  2h = INTB
   *  3h = INTC
   *  4h = INTD
   * Others = Reserved.
   *
   * Field size: 8 bits
   *
   */
  uint8_t intPin;
  /**
   * @brief [rw] Interrupt Line. Value is system software specified.
   * 
   * Field size: 8 bits
   *
   */
  uint8_t intLine;
} pcieType1BridgeIntReg_t;
/* @} */

/*****************************************************************************
 **********  Power Management Capabilities REGISTERS  ************************
 ****************************************************************************/

/**
 * @ingroup pcielld_reg_cfg_pwr_structures
 * @brief Specification of the Power Management Capability Register
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * @{
 */
typedef struct pciePMCapReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] PME Support. 
   *
   * Identifies the power states from which generates PME Messages. A value of 
   * 0 for any bit indicates that the device (or function) is not capable 
   * of generating PME Messages while in that power state. 
   *
   * bit 0x10: If set, PME Messages can be generated from D3cold.\n
   * bit 0x08: If set, PME Messages can be generated from D3hot.\n
   * bit 0x04: If set, PME Messages can be generated from D2.\n
   * bit 0x02: If set, PME Messages can be generated from D1.\n
   * bit 0x01: If set, PME Messages can be generated from D0.
   * 
   * Field size: 5 bits
   *
   */
  uint8_t pmeSuppN;
  /**
   * @brief [rw] D2 Support.
   * 
   * Field size: 1 bit
   *
   */
  uint8_t d2SuppN;
  /**
   * @brief [rw] D1 Support.
   * 
   * Field size: 1 bit
   *
   */
  uint8_t d1SuppN;
  /**
   * @brief [rw] Auxiliary Current
   * 
   * Field size: 3 bits
   *
   */
  uint8_t auxCurrN;
  /**
   * @brief [rw] Device Specific Initialization
   * 
   * Field size: 1 bit
   *
   */
  uint8_t dsiN;
  /**
   * @brief [ro] PME clock.  Hardwired to zero.
   * 
   * Field size: 1 bit
   *
   */
  uint8_t pmeClk;
  /**
   * @brief [rw] Power Management Specification Version
   * 
   * Field size: 3 bits
   *
   */
  uint8_t pmeSpecVer;
  /**
   * @brief [rw] Next capability pointer. 
   *
   * By default, it points to Message Signaled Interrupt structure.
   * 
   * Field size: 8 bits
   *
   */
  uint8_t pmNextPtr;
  /**
   * @brief [rw] Power Management Capability ID.
   * 
   * Field size: 8 bits
   *
   */
  uint8_t pmCapID;
} pciePMCapReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_pwr_structures
 * @brief Specification of the Power Management Capabilities Control and Status Register
 *
 * This register may be used for both endpoint and root complex modes.
 * 
 * @{
 */
typedef struct pciePMCapCtlStatReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [ro] Data register for additional information. Not supported. 
   * 
   * Field size: 8 bits
   *
   */
  uint8_t dataReg;
  /**
   * @brief [ro] Bus Power/Clock Control Enable. Hardwired to zero.
   * 
   * Field size: 1 bits
   *
   */
  uint8_t clkCtrlEn;
  /**
   * @brief [ro] B2 and B3 support. Hardwired to zero.
   * 
   * Field size: 1 bits
   *
   */
  uint8_t b2b3Support;
  /**
   * @brief [rw] PME Status. Indicates if a previously enabled PME event occurred or not.
   * 
   * Write 1 to clear.
   *
   * Field size: 1 bits
   *
   */
  uint8_t pmeStatus;
  /**
   * @brief [ro] Data Scale. Not supported.
   * 
   * Field size: 2 bits
   *
   */
  uint8_t dataScale;
  /**
   * @brief [ro] Data Select. Not supported.
   * 
   * Field size: 4 bits
   *
   */
  uint8_t dataSelect;
  /**
   * @brief [rw] PME Enable. Value of 1 indicates device is enabled to generate PME.
   * 
   * Field size: 1 bits
   *
   */
  uint8_t pmeEn;
  /**
   * @brief [rw] No Soft Reset. 
   *
   * It is set to disable reset during a transition from D3 to D0. 
   * 
   * Field size: 1 bits
   *
   */
  uint8_t noSoftRst;
  /**
   * @brief [rw] Power State.
   *
   * Controls the device power state. Writes are ignored if the state is not
   * supported.
   * 0 = D0 power state
   * 1h = D1 power state
   * 2h = D2 power state
   * 3h = D3 power states
   *
   * Field size: 2 bits
   *
   */
  uint8_t pwrState;
} pciePMCapCtlStatReg_t;
/* @} */

/*****************************************************************************
 **********  Message Signaling Interrupt  REGISTERS  *************************
 ****************************************************************************/

/**
 * @ingroup pcielld_reg_cfg_msi_structures
 * @brief Specification of the MSI capabilities Register
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * @{
 */
typedef struct pcieMsiCapReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] 64bit addressing enabled 
   *
   * Field size: 1 bit
   *
   */
  uint8_t en64bit;
  /**
   * @brief [rw] Multiple Msg enabled 
   *
   * Indicates that multiple message mode is enabled by software. Number
   * of messages enabled must not be greater than @ref multMsgCap
   *
   * <TABLE>
   * <TR><TH>@ref multMsgEn</TH><TH>Number of messages</TH></TR>
   * <TR><TD>0</TD>             <TD>1</TD></TR>
   * <TR><TD>1</TD>             <TD>2</TD></TR>
   * <TR><TD>2</TD>             <TD>4</TD></TR>
   * <TR><TD>3</TD>             <TD>8</TD></TR>
   * <TR><TD>4</TD>             <TD>16</TD></TR>
   * <TR><TD>5</TD>             <TD>32</TD></TR>
   * <TR><TD>others</TD>        <TD>reserved</TD></TR>
   * </TABLE>
   * 
   * Field size: 3 bits
   */
  uint8_t multMsgEn;
  /**
   * @brief [rw] Multipe Msg capable 
   *
   * Multiple message capable.
   * 
   * <TABLE>
   * <TR><TH>@ref multMsgCap</TH><TH>Number of messages</TH></TR>
   * <TR><TD>0</TD>              <TD>1</TD></TR>
   * <TR><TD>1</TD>              <TD>2</TD></TR>
   * <TR><TD>2</TD>              <TD>4</TD></TR>
   * <TR><TD>3</TD>              <TD>8</TD></TR>
   * <TR><TD>4</TD>              <TD>16</TD></TR>
   * <TR><TD>5</TD>              <TD>32</TD></TR>
   * <TR><TD>others</TD>         <TD>reserved</TD></TR>
   * </TABLE>
   *
   * Field size: 3 bits
   */
  uint8_t multMsgCap;
  /**
   * @brief [rw] MSI enabled 
   *
   * MSI Enabled. When set, INTx must be disabled.
   *
   * Field size: 1 bit
   */
  uint8_t msiEn;
  /**
   * @brief [rw] Next capability pointer 
   *
   * By default, it points to PCI Express Capabilities structure.
   * 
   * Field size: 8 bits
   */
  uint8_t nextCap;
  /**
   * @brief [ro] MSI capability ID 
   *
   * Field size: 8 bits
   */
  uint8_t capId;
} pcieMsiCapReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_msi_structures
 * @brief Specification of the MSI lower 32 bits Register
 *
 * This register may be used for both endpoint and root complex modes.
 * 
 * @{
 */
typedef struct pcieMsiLo32Reg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Lower 32bits address
   *
   * Field size: 30 bits
   *
   */
  uint32_t addr;
} pcieMsiLo32Reg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_msi_structures
 * @brief Specification of the MSI upper 32 bits Register
 *
 * This register may be used for both endpoint and root complex modes.
 * 
 * @{
 */
typedef struct pcieMsiUp32Reg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] Upper 32bits address 
   *
   * Field size: 32 bits
   *
   */
  uint32_t addr;
} pcieMsiUp32Reg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_msi_structures
 * @brief Specification of the MSI Data Register
 *
 * This register may be used for both endpoint and root complex modes.
 * 
 * @{
 */
typedef struct pcieMsiDataReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /**
   * @brief [rw] MSI data 
   *
   * Field size: 16 bits
   *
   */
  uint16_t data;
} pcieMsiDataReg_t;
/* @} */

/*****************************************************************************
 **********  PCIe CAPABILITIES  REGISTERS ************************************
 ****************************************************************************/
/** 
 * @ingroup pcielld_reg_cfg_cap_structures 
 * @brief Specification of the PCI Express Capabilities Register 
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * @{
 */
typedef struct pciePciesCapReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */  
  /** 
   * @brief [rw] Interrupt Message Number. Updated by hardware and writable through internal bus Interface.     
   *            
   * Field size: 5 bits
   */
  uint8_t intMsg;
  /** 
   * @brief [rw] Slot Implemented. Writable from internal bus interface.          
   *            
   * Field size: 1 bit
   */
  uint8_t sltImplN;
  /** 
   * @brief [rw] Device Port Type.              
   *
   * 0 = EP type\n
   * 4h = RC type\n
   * Others = Reserved
   * 
   * Field size: 4 bits
   */
  uint8_t dportType;
  /** 
   * @brief [rw] PCI Express Capability Version             
   *            
   * Field size: 4 bits
   */
  uint8_t pcieCap;
  /** 
   * @brief [rw] Next capability pointer. Writable from internal bus interface.         
   *            
   * Field size: 8 bits
   */
  uint8_t nextCap;
  /** 
   * @brief [rw] PCIe Capability ID.              
   *            
   * Field size: 8 bits
   */
  uint8_t capId;
} pciePciesCapReg_t;
/* @} */

/** 
 * @ingroup pcielld_reg_cfg_cap_structures 
 * @brief Specification of the Device Capabilities Register 
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * @{
 */
typedef struct pcieDeviceCapReg_s {                            
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */  
  /** 
   * @brief [rw] Captured Slot Power Limit Scale. For upstream ports (EP ports) only.      
   *            
   * Field size: 2 bits
   */
  uint8_t pwrLimitScale;
  /** 
   * @brief [rw] Captured Slow Power Limit Value. For upstream ports (EP ports) only.      
   *            
   * Field size: 8 bits
   */
  uint8_t pwrLimitValue;
  /** 
   * @brief [rw] Role-based Error Reporting. Writable from internal bus interface.         
   *            
   * Field size: 1 bit
   */
  uint8_t errRpt;
  /** 
   * @brief [rw] Endpoint L1 Acceptable Latency. Must be 0 in RC mode. It is 3h for EP mode. 
   *            
   * Field size: 3 bits
   */
  uint8_t l1Latency;
  /** 
   * @brief [rw] Endpoint L0s Acceptable Latency. Must be 0 in RC mode. It is 4h for EP mode. 
   *            
   * Field size: 3 bits
   */
  uint8_t l0Latency;
  /** 
   * @brief [rw] Extended Tag Field Supported. Writable from internal interface         
   *            
   * Field size: 1 bit
   */
  uint8_t extTagFld;
  /** 
   * @brief [rw] Phantom Field Supported. Writable from internal bus interface.         
   *            
   * Field size: 2 bits
   */
  uint8_t phantomFld;
  /** 
   * @brief [rw] Maximum Payload size supported. Writable from internal bus interface.        
   *            
   * Field size: 3 bits
   */
  uint8_t maxPayldSz;
} pcieDeviceCapReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_cap_structures
 * @brief Specification of the Device Status and Control Register
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * @{
 */
typedef struct pcieDevStatCtrlReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /** 
   * @brief [ro] Transaction Pending 
   *
   * Field size: 1 bit
   */
  uint8_t tpend;
  /** 
   * @brief [ro] Auxiliary Power Detected 
   *
   * Field size: 1 bit
   */
  uint8_t auxPwr;
  /** 
   * @brief [rw] Unsupported Request Detected 
   *
   * Field size: 1 bit
   */
  uint8_t rqDet;
  /** 
   * @brief [rw] Fatal Error Detected 
   *
   * Field size: 1 bit
   */
  uint8_t fatalEr;
  /** 
   * @brief [rw] Non-fatal Error Detected 
   *
   * Field size: 1 bit
   */
  uint8_t nFatalEr;
  /** 
   * @brief [rw] Correctable Error Detected 
   *
   * Field size: 1 bit
   */
  uint8_t corrEr;
  /** 
   * @brief [rw] Maximum Read Request Size 
   *
   * Field size: 3 bits
   */
  uint8_t maxSz;
  /** 
   * @brief [rw] Enable no snoop 
   *
   * Field size: 1 bit
   */
  uint8_t noSnoop;
  /** 
   * @brief [rw] AUX Power PM Enable 
   *
   * Field size: 1 bit
   */
  uint8_t auxPwrEn;
  /** 
   * @brief [rw] Phantom Function Enable 
   *
   * Field size: 1 bit
   */
  uint8_t phantomEn;
  /** 
   * @brief [rw] Extended Tag Field Enable 
   *
   * Field size: 1 bit
   */
  uint8_t xtagEn;
  /** 
   * @brief [rw] Maximum Payload Size 
   *
   * Field size: 3 bits
   */
  uint8_t maxPayld;
  /** 
   * @brief [rw] Enable Relaxed Ordering 
   *
   * Field size: 1 bit
   */
  uint8_t relaxed;
  /** 
   * @brief [rw] Enable Unsupported Request Reporting 
   *
   * Field size: 1 bit
   */
  uint8_t reqRp;
  /** 
   * @brief [rw] Fatal Error Reporting Enable 
   *
   * Field size: 1 bit
   */
  uint8_t fatalErRp;
  /** 
   * @brief [rw] Non-fatal Error Reporting Enable 
   *
   * Field size: 1 bit
   */
  uint8_t nFatalErRp;
  /** 
   * @brief [rw] Correctable Error Reporting Enable 
   *
   * Field size: 1 bit
   */
  uint8_t corErRp;
} pcieDevStatCtrlReg_t;
/* @} */

/** 
 *  @ingroup pcielld_reg_cfg_cap_structures 
 *  @brief Specification of the Link Capabilities Register 
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * @{
 */
typedef struct pcieLinkCapReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */  
  /** 
   * @brief [rw] Port Number. Writable from internal bus interface.          
   *            
   * Field size: 8 bits
   */
  uint8_t portNum;
  /** 
   * @brief [rw] Bandwidth Notification Capable.              
   *
   * 0 = For upstream ports (EP ports)\n
   * 1 = For downstream ports (RC ports)
   *
   * Field size: 1 bit
   */
  uint8_t bwNotifyCap;
  /** 
   * @brief [rw] Link Layer Active Reporting Capable.            
   *
   * 0 = For upstream ports (EP ports)\n
   * 1 = For downstream ports (RC ports)
   *
   * Field size: 1 bit
   */
  uint8_t dllRepCap;
  /** 
   * @brief [rw] Surprise Down Error Reporting Capable. Not supported. Always zero.        
   *            
   * Field size: 1 bit
   */
  uint8_t downErrRepCap;
  /** 
   * @brief [rw] Clock Power Management. Writable from internal bus interface.         
   *
   * For upstream ports (EP Ports), a value of 1h in this bit indicates that 
   * the component tolerates the removal of any reference clock(s) in the L1 
   * and L2/L3 Ready Link states. A value of 0 indicates the reference 
   * clock(s) must not be removed in these Link states.
   * 
   * For downstream ports (RC Ports), this bit is always 0.
   *
   * Field size: 1 bit
   */
  uint8_t clkPwrMgmt;
  /** 
   * @brief [rw] L1 Exit Latency when common clock is used. Writable from internal bus interface.    
   *
   * <TABLE>
   * <TR><TH>@ref l1ExitLat</TH><TH>low range</TH><TH>high range</TH></TR>
   * <TR><TD>0</TD>             <TD>0</TD>        <TD>64 ns</TD></TR>
   * <TR><TD>1</TD>             <TD>64ns</TD>     <TD>128ns</TD></TR>
   * <TR><TD>2</TD>             <TD>128ns</TD>    <TD>256ns</TD></TR>
   * <TR><TD>3</TD>             <TD>256ns</TD>    <TD>512ns</TD></TR>
   * <TR><TD>4</TD>             <TD>512ns</TD>    <TD>1µs</TD></TR>
   * <TR><TD>5</TD>             <TD>1µs</TD>      <TD>2µs</TD></TR>
   * <TR><TD>6</TD>             <TD>2µs</TD>      <TD>4µs</TD></TR>
   * <TR><TD>7</TD>             <TD>4µs</TD>      <TD>and up</TD></TR>
   * </TABLE>
   *            
   * Field size: 3 bits
   */
  uint8_t l1ExitLat;
  /** 
   * @brief [rw] L0s Exit Latency. Writable from internal bus interface.         
   *
   * <TABLE>
   * <TR><TH>@ref l1ExitLat</TH><TH>low range</TH><TH>high range</TH></TR>
   * <TR><TD>0</TD>             <TD>0</TD>        <TD>64 ns</TD></TR>
   * <TR><TD>1</TD>             <TD>64ns</TD>     <TD>128ns</TD></TR>
   * <TR><TD>2</TD>             <TD>128ns</TD>    <TD>256ns</TD></TR>
   * <TR><TD>3</TD>             <TD>256ns</TD>    <TD>512ns</TD></TR>
   * <TR><TD>4</TD>             <TD>512ns</TD>    <TD>1µs</TD></TR>
   * <TR><TD>5</TD>             <TD>1µs</TD>      <TD>2µs</TD></TR>
   * <TR><TD>6</TD>             <TD>2µs</TD>      <TD>4µs</TD></TR>
   * <TR><TD>7</TD>             <TD>4µs</TD>      <TD>and up</TD></TR>
   * </TABLE>
   *
   * Field size: 3 bits
   */
  uint8_t losExitLat;
  /** 
   * @brief [rw] Active State Link Power Management Support. Writable from internal bus interface.      
   *
   * 1h = L0s entry supported.\n
   * 3h = L0s and L1 supported.\n
   * Others = Reserved.
   *
   * Field size: 2 bits
   */
  uint8_t asLinkPm;
  /** 
   * @brief [rw] Maximum Link Width. Writable from internal bus interface.   
   *
   * 1h = ×1\n
   * 2h = ×2\n
   * Others = Reserved.
   * 
   * Field size: 6 bits
   */
  uint8_t maxLinkWidth;
  /** 
   * @brief [rw] Maximum Link Speed. Writable from internal bus interface.         
   *
   * 1h = 2.5GT/s Link speed supported.\n
   * 2h = 5.0 GT/s and 2.5 GT/s Link speeds supported.\n
   * Others = Reserved.
   * 
   * Field size: 4 bits
   */
  uint8_t maxLinkSpeed;
} pcieLinkCapReg_t;
/* @} */

/** 
 * @ingroup pcielld_reg_cfg_cap_structures 
 * @brief Specification of the Link Status and Control Register 
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * @{
 */
typedef struct pcieLinkStatCtrlReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */  
  /** 
   * @brief [rw] Link Autonomous Bandwidth Status.
   *
   * This bit is Set by hardware to indicate that hardware has autonomously 
   * changed Link speed or width, without the Port transitioning through 
   * DL_Down status, for reasons other than to attempt to correct unreliable 
   * Link operation.  This bit must be set if the Physical Layer reports a 
   * speed or width change was initiated by the downstream component that 
   * was indicated as an autonomous change.
   * 
   * Not applicable and reserved for EP.
   *
   * Field size: 1 bit
   */
  uint8_t linkBwStatus;
  /** 
   * @brief [rw] Link Bandwidth Management Status.                         
   *
   * This bit is Set by hardware to indicate that either of the following 
   * has occurred without the Port transitioning through DL_Down status:
   *
   * - A Link retraining has completed following a write of 1b to the 
   *   Retrain Link bit
   * - Hardware has changed Link speed or width to attempt to correct 
   *   unreliable Link operation, either through an LTSSM timeout or 
   *   a higher level process.
   *
   * This bit must be set if the Physical Layer reports a speed or width 
   * change was initiated by the downstream component that was not 
   * indicated as an autonomous change.
   * 
   * Not applicable and reserved for EP.
   *            
   * Field size: 1 bit
   */
  uint8_t linkBwMgmtStatus;
  /** 
   * @brief [rw] Data Link Layer Active                         
   *            
   * This bit indicates the status of the Data Link Control and 
   * Management State Machine. It returns a 1 to indicate the DL_Active state,
   * 0 otherwise.
   *            
   * Field size: 1 bit
   */
  uint8_t dllActive;
  /** 
   * @brief [rw] Slot Clock Configuration. Writable from internal bus interface.                     
   *            
   * This bit indicates that the component uses the same 
   * physical reference clock that the platform provides on the connector.
   *            
   * Field size: 1 bit
   */
  uint8_t slotClkCfg;
  /** 
   * @brief [rw] Link Training. Not applicable to EP.                       
   *            
   * Field size: 1 bit
   */
  uint8_t linkTraining;
  /** 
   * @brief [rw] Undefined for PCI Express.                         
   *            
   * Field size: 1 bit
   */
  uint8_t undef;
  /** 
   * @brief [rw] Negotiated Link Width. Set automatically by hardware after link initialization.                   
   *            
   * Field size: 6 bits
   */
  uint8_t negotiatedLinkWd;
  /** 
   * @brief [rw] Link Speed. Set automatically by hardware after link initialization.                    
   *            
   * Field size: 4 bits
   */
  uint8_t linkSpeed;
  /** 
   * @brief [rw]  Link Autonomous Bandwidth Interrupt Enable. Not applicable and is reserved for EP              
   *            
   * Field size: 1 bit
   */
  uint8_t linkBwIntEn;
  /** 
   * @brief [rw] Link Bandwidth Management Interrupt Enable. Not applicable and is reserved for EP.              
   *            
   * Field size: 1 bit
   */
  uint8_t linkBwMgmtIntEn;
  /** 
   * @brief [rw] Hardware Autonomous Width Disable. Not supported and hardwired to zero.                           
   *            
   * Field size: 1 bit
   */
  uint8_t hwAutoWidthDis;
  /** 
   * @brief [rw] Enable Clock Power Management.                          
   *            
   * Field size: 1 bit
   */
  uint8_t clkPwrMgmtEn;
  /** 
   * @brief [rw] Extended Synchronization.              
   *            
   * Field size: 1 bit
   */
  uint8_t extSync;
  /** 
   * @brief [rw] Common Clock Configuration.                   
   *
   * 0 = Indicates that this device and the device at the opposite end of the 
   * link are operating with separate reference clock sources.\n
   * 1 = Indicates that this device and the device at the opposite end of the
   * link are operating with a common clock source.
   * 
   * Field size: 1 bit
   */
  uint8_t commonClkCfg;
  /** 
   * @brief [rw] Retrain Link. Not applicable and reserved for EP.                       
   *            
   * Field size: 1 bit
   */
  uint8_t retrainLink;
  /** 
   * @brief [rw] Disables the link by directing the LTSSM to the Disabled state when set.
   *            
   * Field size: 1 bit
   */
  uint8_t linkDisable;
  /** 
   * @brief [rw] Read Completion Boundary.
   *
   * 0 = 64 bytes\n
   * 1 = 128 bytes
   * 
   * Field size: 1 bit
   */
  uint8_t rcb;
  /** 
   * @brief [rw] Active State Link Power Management Control
   *
   * 0 = Disabled.\n
   * 1h = L0s entry enabled.\n
   * 2h = L1 entry enabled.\n
   * 3h = L0s and L1 entry enabled.\n
   * 
   * Field size: 2 bits
   */
  uint8_t activeLinkPm;
} pcieLinkStatCtrlReg_t;
/* @} */

/** 
 * @ingroup pcielld_reg_cfg_cap_structures 
 * @brief Specification of the Slot Capabilities register
 *
 * This register may only be used for root complex mode.
 *
 * @{
 */
typedef struct pcieSlotCapReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */  
  /** 
   * @brief [rw] Physical Slot Number.
   *
   * Field size: 13 bits [0-0x1FFF]
   */
  uint16_t slotNum;
  /** 
   * @brief [rw] No Command Complete Support
   *
   * When Set, this bit indicates that this slot does not generate software 
   * notification when an issued command is completed by the Hot-Plug Controller
   * 
   * Field size: 1 bit
   */
  uint8_t cmdCompSupp;
  /** 
   * @brief [rw] Electromechanical Interlock Present.
   *
   * When Set, this bit indicates that an Electromechanical Interlock 
   * is implemented on the chassis for this slot.
   * 
   * Field size: 1 bit
   */
  uint8_t emlPresent;
  /** 
   * @brief [rw] Slot Power Limit Scale.
   *
   * Field size: 2 bits
   */
  uint8_t pwrLmtScale;
  /** 
   * @brief [rw] Slot Power Limit Value.
   *
   * Field size: 8 bits
   */
  uint8_t pwrLmtValue;
  /** 
   * @brief [rw] Hot Plug Capable.
   *
   * Field size: 1 bit
   */
  uint8_t hpCap;
  /** 
   * @brief [rw] Hot Plug Surprise.
   *
   * Field size: 1 bit
   */
  uint8_t hpSurprise;
  /** 
   * @brief [rw] Power Indicator Present.
   *
   * Field size: 1 bit
   */
  uint8_t pwrInd;
  /** 
   * @brief [rw] Attention Indicator Present.
   *
   * Field size: 1 bit
   */
  uint8_t attnInd;
  /** 
   * @brief [rw] MRL Sensor Present.
   *
   * Field size: 1 bit
   */
  uint8_t mrlSensor;
  /** 
   * @brief [rw] Power Controller Present.
   *
   * If there is no power controller, software must ensure that system power 
   * is up before reading Presence Detect state
   * 
   * Field size: 1 bit
   */
  uint8_t pwrCtl;
  /** 
   * @brief [rw] Attention Indicator Present.
   *
   * Field size: 1 bit
   */
  uint8_t attnButton;
} pcieSlotCapReg_t;
/* @} */

/** 
 * @ingroup pcielld_reg_cfg_cap_structures 
 * @brief Specification of the Slot Status and Control register
 *
 * This register may only be used for root complex mode.
 *
 * @{
 */
typedef struct pcieSlotStatCtrlReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */  
  /** 
   * @brief [rw] Data Link Layer State Changed 
   *
   * Write 1 to clear.
   *
   * Field size: 1 bit
   */
  uint8_t dllState;
  /** 
   * @brief [ro] Electromechanical Lock Status
   *
   * Field size: 1 bit
   */
  uint8_t emLock;
  /** 
   * @brief [ro] Presence Detect State
   *
   * Field size: 1 bit
   */
  uint8_t presenceDet;
  /** 
   * @brief [ro] MRL Sensor State
   *
   * Field size: 1 bit
   */
  uint8_t mrlState;
  /** 
   * @brief [rw] Command Completed
   *
   * Write 1 to clear.
   *
   * Field size: 1 bit
   */
  uint8_t cmdComplete;
  /** 
   * @brief [rw] Presence Detect Changed
   *
   * Write 1 to clear.
   *
   * Field size: 1 bit
   */
  uint8_t presenceChg;
  /** 
   * @brief [rw] MRL Sensor Changed
   *
   * Write 1 to clear.
   *
   * Field size: 1 bit
   */
  uint8_t mrlChange;
  /** 
   * @brief [rw] Power Fault Detected
   *
   * Write 1 to clear.
   *
   * Field size: 1 bit
   */
  uint8_t pwrFault;
  /** 
   * @brief [rw] Attention Button Pressed.
   *
   * Write 1 to clear.
   *
   * Field size: 1 bit
   */
  uint8_t attnPressed;
  /** 
   * @brief [rw] Data Link Layer State Changed Enable.
   *
   * Field size: 1 bit
   */
  uint8_t dllChgEn;
  /** 
   * @brief [rw] Electromechanical Interlock Control.
   *
   * Field size: 1 bit
   */
  uint8_t emLockCtl;
  /** 
   * @brief [rw] Power Controller Control
   *
   * Field size: 1 bit
   */
  uint8_t pmCtl;
  /** 
   * @brief [rw] Power Indicator Control
   *
   * Field size: 2 bits
   */
  uint8_t pmIndCtl;
  /** 
   * @brief [rw] Attention Indicator Control.
   *
   * Field size: 2 bits
   */
  uint8_t attnIndCtl;
  /** 
   * @brief [rw] Hot Plug Interrupt Enable.
   *
   * Field size: 1 bit
   */
  uint8_t hpIntEn;
  /** 
   * @brief [rw] Command Completed Interrupt Enable.
   *
   * Field size: 1 bit
   */
  uint8_t cmdCmpIntEn;
  /** 
   * @brief [rw] Presence Detect Changed Enable.
   *
   * Field size: 1 bit
   */
  uint8_t prsDetChgEn;
  /** 
   * @brief [rw] MRL Sensor Changed Enable.
   *
   * Field size: 1 bit
   */
  uint8_t mrlChgEn;
  /** 
   * @brief [rw] Power Fault Detected Enable.
   *
   * Field size: 1 bit
   */
  uint8_t pwrFltDetEn;
  /** 
   * @brief [rw] Attention Button Pressed Enable.
   *
   * Field size: 1 bit
   */
  uint8_t attnButtEn;
} pcieSlotStatCtrlReg_t;
/* @} */

/** 
 * @ingroup pcielld_reg_cfg_cap_structures 
 * @brief Specification of the Root Control and Capabilities Register
 *
 * This register may only be used for root complex mode.
 *
 * @{
 */
typedef struct pcieRootCtrlCapReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */  
  /** 
   * @brief [ro] CRS Software Visibility. Not supported and set to 0.
   *
   * Field size: 1 bit
   */
  uint8_t crsSw;
  /** 
   * @brief [ro] CRS Software Visibility Enable. Not supported and set to 0x0.
   *
   * Field size: 1 bit
   */
  uint8_t crsSwEn;
  /** 
   * @brief [rw] PME Interrupt Enable
   *
   * Field size: 1 bit
   */
  uint8_t pmeIntEn;
  /** 
   * @brief [rw] System Error on Fatal Error Enable
   *
   * Field size: 1 bit
   */
  uint8_t serrFatalErr;
  /** 
   * @brief [rw] System Error on Non-fatal Error Enable
   *
   * Field size: 1 bit
   */
  uint8_t serrNFatalErr;
  /** 
   * @brief [rw] System Error on Correctable Error Enable
   *
   * Field size: 1 bit
   */
  uint8_t serrEn;
} pcieRootCtrlCapReg_t;
/* @} */

/** 
 * @ingroup pcielld_reg_cfg_cap_structures 
 * @brief Specification of the Root Status and Control register
 *
 * This register may only be used for root complex mode.
 *
 * @{
 */
typedef struct pcieRootStatusReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */  
  /** 
   * @brief [ro] Indicates that another PME is pending when the PME Status bit is Set.
   *
   * Field size: 1 bit
   */
  uint8_t pmePend;
  /** 
   * @brief [rw] Indicates that PME was asserted by the PME Requester.
   *
   * Write 1 to clear
   *
   * Field size: 1 bit
   */
  uint8_t pmeStatus;
  /** 
   * @brief [ro] ID of the last PME Requester.
   *
   * This field is only valid when the PME Status bit is Set.
   *
   * Field size: 16 bits
   */
  uint16_t pmeReqID;
} pcieRootStatusReg_t;
/* @} */


/** 
 * @ingroup pcielld_reg_cfg_cap_structures 
 * @brief Specification of the Device Capabilities 2 Register 
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * @{
 */
typedef struct pcieDevCap2Reg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */  
  /** 
   * @brief [rw] Completion timeout disable supported                         
   *            
   * Field size: 1 bit
   */
  uint8_t cmplToDisSupp;
  /** 
   * @brief [rw] Completion timeout ranges supported. Applicable to RC/EP that issue requests on own behalf.                
   *            
   * Field size: 4 bits
   */
  uint8_t cmplToEn;
} pcieDevCap2Reg_t;
/* @} */

/** 
 * @ingroup pcielld_reg_cfg_cap_structures 
 * @brief Specification of the Device Status and Control Register 2 
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * @{
 */
typedef struct pcieDevStatCtrl2Reg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */  
  /** 
   * @brief [rw] Completion timeout disable                          
   *            
   * Field size: 1 bit
   */
  uint8_t cmplToDis;
  /** 
   * @brief [rw] Completion timeout value.                          
   *
   * It is strongly recommended that the Completion Timeout mechanism 
   * not expire in less than 10 ms.
   * 
   * <TABLE>
   * <TR><TH>@ref cmplTo</TH><TH>low range</TH><TH>high range</TH></TR>
   * <TR><TD>0x0</TD>        <TD>50ms</TD>     <TD>50s</TD></TR>
   * <TR><TD>0x1</TD>        <TD>50s</TD>      <TD>100s</TD></TR>
   * <TR><TD>0x2</TD>        <TD>1ms</TD>      <TD>10ms</TD></TR>
   * <TR><TD>0x5</TD>        <TD>16ms</TD>     <TD>55ms</TD></TR>
   * <TR><TD>0x6</TD>        <TD>65ms</TD>     <TD>210ms</TD></TR>
   * <TR><TD>0x9</TD>        <TD>260ms</TD>    <TD>900ms</TD></TR>
   * <TR><TD>0xA</TD>        <TD>1s</TD>       <TD>3.5s</TD></TR>
   * <TR><TD>0xD</TD>        <TD>4s</TD>       <TD>13s</TD></TR>
   * <TR><TD>0xE</TD>        <TD>17s</TD>      <TD>64s</TD></TR>
   * <TR><TD>others</TD>     <TD>reserved</TD> <TD>reserved</TD></TR>
   * </TABLE>
   *
   * Field size: 4 bits
   */
  uint8_t cmplTo;
} pcieDevStatCtrl2Reg_t;
/* @} */

/** 
 * @ingroup pcielld_reg_cfg_cap_structures 
 * @brief Specification of the Link Control 2 Register 
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * @{
 */
typedef struct pcieLinkCtrl2Reg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */  
  /** 
   * @brief [rw] Current De-emphasis level                          
   *
   * 0 = -6 dB\n
   * 1 = -3.5 dB
   *
   * Field size: 1 bit
   */
  uint8_t deEmph;
  /** 
   * @brief [rw] De-emphasis level in polling-compliance state
   *
   * This bit sets the de-emphasis level in Polling Compliance state if the 
   * entry occurred due to the Enter Compliance bit being 1.
   *
   * 0 = -6 dB\n
   * 1 = -3.5 dB
   * 
   * Field size: 1 bit
   */
  uint8_t pollDeemph;
  /** 
   * @brief [rw] Compliance SOS.                           
   *
   * When this bit is set to 1, the LTSSM is required to send SKP 
   * Ordered Sets periodically in between the modified compliance patterns.
   * 
   * Field size: 1 bit
   */
  uint8_t cmplSos;
  /** 
   * @brief [rw] Enter modified compliance.                          
   *
   * When this bit is set to 1, the device transmits Modified Compliance 
   * Pattern if the LTSSM enters Polling Compliance substate.
   * 
   * Field size: 1 bit
   */
  uint8_t entrModCompl;
  /** 
   * @brief [rw] Value of non-de-emphasized voltage level at transmitter pins.                     
   *            
   * Field size: 3 bits
   */
  uint8_t txMargin;
  /** 
   * @brief [rw] Selectable De-emphasis.                           
   *
   * When the Link is operating at 5.0 GT/s speed, this bit selects the level 
   * of de-emphasis for an upstream component.  When the Link is operating at 
   * 2.5 GT/s speed, the setting of this bit has no effect.
   *
   * 0 = -6 dB\n
   * 1 = -3.5 dB
   * 
   * Field size: 1 bit
   */
  uint8_t selDeemph;
  /** 
   * @brief [rw] Hardware Autonomous Speed Disable.                         
   *
   * 0 = Enable hardware to change the link speed.\n
   * 1 = Disables hardware from changing the Link speed for device specific 
   * reasons other than attempting to correct unreliable Link operation by 
   * reducing Link speed.
   * 
   * Field size: 1 bit
   */
  uint8_t hwAutoSpeedDis;
  /** 
   * @brief [rw] Enter Compliance.                           
   *
   * Software is permitted to force a Link to enter Compliance mode at the 
   * speed indicated in the Target Link Speed field by setting this bit to 
   * 1 in both components on a Link and then initiating a hot reset on the Link.
   * 
   * Field size: 1 bit
   */
  uint8_t entrCompl;
  /** 
   * @brief [rw] Target Link Speed.                          
   *
   * 1h = 2.5 GT/s Target Link Speed.\n
   * 2h = 5.0 GT/s Target Link Speed.\n
   * Others = Reserved.
   *
   * Field size: 4 bits
   */
  uint8_t tgtSpeed;
} pcieLinkCtrl2Reg_t;
/* @} */


/*****************************************************************************
 **********  PCIe EXTENDED CAPABILITIES  REGISTERS ***************************
 ****************************************************************************/
/** 
 * @ingroup pcielld_reg_cfg_cap_ext_structures 
 * @brief Specification of the Extended Capabilities Header register
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * @{
 */
typedef struct pcieExtCapReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */  
  /** 
   * @brief [ro] Next Capability Pointer
   *
   * Field size: 12 bits
   */
  uint16_t nextCap;
  /** 
   * @brief [ro] Extended Capability Version
   *
   * Field size: 4 bits
   */
  uint8_t extCapVer;
  /** 
   * @brief [ro] PCIe Extended Capability ID
   *
   * Field size: 16 bits
   */
  uint16_t extCapID;
} pcieExtCapReg_t;
/* @} */

/** 
 * @ingroup pcielld_reg_cfg_cap_ext_structures 
 * @brief Specification of the Uncorrectable Error Status register
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * @{
 */
typedef struct pcieUncErrReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */  
  /** 
   * @brief [rw] Unsupported Request Error Status
   *
   * Write 1 to clear
   *
   * Field size: 1 bit
   */
  uint8_t urErrSt;
  /** 
   * @brief [rw] ECRC Error Status
   *
   * Write 1 to clear
   *
   * Field size: 1 bit
   */
  uint8_t ecrcErrSt;
  /** 
   * @brief [rw] Malformed TLP Status
   *
   * Write 1 to clear
   *
   * Field size: 1 bit
   */
  uint8_t mtlpErrSt;
  /** 
   * @brief [rw] Receiver Overflow Status
   *
   * Write 1 to clear
   *
   * Field size: 1 bit
   */
  uint8_t rcvrOfSt;
  /** 
   * @brief [rw] Unexpected Completion Status
   *
   * Write 1 to clear
   *
   * Field size: 1 bit
   */
  uint8_t ucmpSt;
  /** 
   * @brief [rw] Completer Abort Status
   *
   * Write 1 to clear
   *
   * Field size: 1 bit
   */
  uint8_t cmplAbrtSt;
  /** 
   * @brief [rw] Completion Timeout Status
   *
   * Write 1 to clear
   *
   * Field size: 1 bit
   */
  uint8_t cmplTmotSt;
  /** 
   * @brief [rw] Flow Control Protocol Error Status
   *
   * Write 1 to clear
   *
   * Field size: 1 bit
   */
  uint8_t fcpErrSt;
  /** 
   * @brief [rw] Poisoned TLP Status
   *
   * Write 1 to clear
   *
   * Field size: 1 bit
   */
  uint8_t psndTlpSt;
  /** 
   * @brief [ro] Surprise Down Error Status. Not supported (always 0)
   *
   * Field size: 1 bit
   */
  uint8_t srpsDnSt;
  /** 
   * @brief [rw] Data Link Protocol Error Status
   *
   * Write 1 to clear
   *
   * Field size: 1 bit
   */
  uint8_t dlpErrSt;
} pcieUncErrReg_t;
/* @} */

/** 
 * @ingroup pcielld_reg_cfg_cap_ext_structures 
 * @brief Specification of the Uncorrectable Error Mask register
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * @{
 */
typedef struct pcieUncErrMaskReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */  
  /** 
   * @brief [rw] Unsupported Request Error Mask
   *
   * Field size: 1 bit
   */
  uint8_t urErrMsk;
  /** 
   * @brief [rw] ECRC Error Mask
   *
   * Field size: 1 bit
   */
  uint8_t ecrcErrMsk;
  /** 
   * @brief [rw] Malformed TLP Mask
   *
   * Field size: 1 bit
   */
  uint8_t mtlpErrMsk;
  /** 
   * @brief [rw] Receiver Overflow Mask
   *
   * Field size: 1 bit
   */
  uint8_t rcvrOfMsk;
  /** 
   * @brief [rw] Unexpected Completion Mask
   *
   * Field size: 1 bit
   */
  uint8_t ucmpMsk;
  /** 
   * @brief [rw] Completer Abort Mask
   *
   * Field size: 1 bit
   */
  uint8_t cmplAbrtMsk;
  /** 
   * @brief [rw] Completion Timeout Mask
   *
   * Field size: 1 bit
   */
  uint8_t cmplTmotMsk;
  /** 
   * @brief [rw] Flow Control Protocol Error Mask
   *
   * Field size: 1 bit
   */
  uint8_t fcpErrMsk;
  /** 
   * @brief [rw] Poisoned TLP Mask
   *
   * Field size: 1 bit
   */
  uint8_t psndTlpMsk;
  /** 
   * @brief [ro] Surprise Down Error Mask. Not supported (always 0)
   *
   * Field size: 1 bit
   */
  uint8_t srpsDnMsk;
  /** 
   * @brief [rw] Data Link Protocol Error Mask
   *
   * Field size: 1 bit
   */
  uint8_t dlpErrMsk;
} pcieUncErrMaskReg_t;
/* @} */

/** 
 * @ingroup pcielld_reg_cfg_cap_ext_structures 
 * @brief Specification of the Uncorrectable Error Severity register
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * Set each bit to 0 to indicate the error is non-fatal
 * Set each bit to 1 to indicate the error is fatal.
 *
 * @{
 */
typedef struct pcieUncErrSvrtyReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */  
  /** 
   * @brief [rw] Unsupported Request Error Severity
   *
   * 0=Non-Fatal; 1=Fatal
   *
   * Field size: 1 bit
   */
  uint8_t urErrSvrty;
  /** 
   * @brief [rw] ECRC Error Severity
   *
   * 0=Non-Fatal; 1=Fatal
   *
   * Field size: 1 bit
   */
  uint8_t ecrcErrSvrty;
  /** 
   * @brief [rw] Malformed TLP Severity
   *
   * 0=Non-Fatal; 1=Fatal
   *
   * Field size: 1 bit
   */
  uint8_t mtlpErrSvrty;
  /** 
   * @brief [rw] Receiver Overflow Severity
   *
   * 0=Non-Fatal; 1=Fatal
   *
   * Field size: 1 bit
   */
  uint8_t rcvrOfSvrty;
  /** 
   * @brief [rw] Unexpected Completion Severity
   *
   * 0=Non-Fatal; 1=Fatal
   *
   * Field size: 1 bit
   */
  uint8_t ucmpSvrty;
  /** 
   * @brief [rw] Completer Abort Severity
   *
   * 0=Non-Fatal; 1=Fatal
   *
   * Field size: 1 bit
   */
  uint8_t cmplAbrtSvrty;
  /** 
   * @brief [rw] Completion Timeout Severity
   *
   * 0=Non-Fatal; 1=Fatal
   *
   * Field size: 1 bit
   */
  uint8_t cmplTmotSvrty;
  /** 
   * @brief [rw] Flow Control Protocol Error Severity
   *
   * 0=Non-Fatal; 1=Fatal
   *
   * Field size: 1 bit
   */
  uint8_t fcpErrSvrty;
  /** 
   * @brief [rw] Poisoned TLP Severity
   *
   * 0=Non-Fatal; 1=Fatal
   *
   * Field size: 1 bit
   */
  uint8_t psndTlpSvrty;
  /** 
   * @brief [ro] Surprise Down Error Severity. Not supported (always 0)
   *
   * 0=Non-Fatal; 1=Fatal
   *
   * Field size: 1 bit
   */
  uint8_t srpsDnSvrty;
  /** 
   * @brief [rw] Data Link Protocol Error Severity
   *
   * 0 = Non-fatal; 1 = Fatal
   *
   * Field size: 1 bit
   */
  uint8_t dlpErrSvrty;
} pcieUncErrSvrtyReg_t;
/* @} */

/** 
 * @ingroup pcielld_reg_cfg_cap_ext_structures 
 * @brief Specification of the Correctable Error Status register
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * @{
 */
typedef struct pcieCorErrReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */  
  /** 
   * @brief [rw] Advisory Non-Fatal Error Status
   *
   * This bit is Set by default to enable compatibility with software 
   * that does not comprehend Role-Based Error Reporting.
   *
   * Write 1 to clear
   *
   * Field size: 1 bit
   */
  uint8_t advNFErrSt;
  /** 
   * @brief [rw] Replay Timer Timeout Status
   *
   * Write 1 to clear
   *
   * Field size: 1 bit
   */
  uint8_t rplyTmrSt;
  /** 
   * @brief [rw] REPLAY_NUM Rollover Status
   *
   * Write 1 to clear
   *
   * Field size: 1 bit
   */
  uint8_t rpltRoSt;
  /** 
   * @brief [rw] Bad DLLP Status
   *
   * Write 1 to clear
   *
   * Field size: 1 bit
   */
  uint8_t badDllpSt;
  /** 
   * @brief [rw] Bad TLP Status
   *
   * Write 1 to clear
   *
   * Field size: 1 bit
   */
  uint8_t badTlpSt;
  /** 
   * @brief [rw] Receiver Error Status
   *
   * Write 1 to clear
   *
   * Field size: 1 bit
   */
  uint8_t rcvrErrSt;
} pcieCorErrReg_t;
/* @} */

/** 
 * @ingroup pcielld_reg_cfg_cap_ext_structures 
 * @brief Specification of the Correctable Error Mask register
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * @{
 */
typedef struct pcieCorErrMaskReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */  
    /** 
   * @brief [rw] Advisory Non-Fatal Error Mask
   *
   * Field size: 1 bit
   */
  uint8_t advNFErrMsk;
  /** 
   * @brief [rw] Replay Timer Timeout Mask
   *
   * Field size: 1 bit
   */
  uint8_t rplyTmrMsk;
  /** 
   * @brief [rw] REPLAY_NUM Rollover Mask
   *
   * Field size: 1 bit
   */
  uint8_t rpltRoMsk;
  /** 
   * @brief [rw] Bad DLLP Mask
   *
   * Field size: 1 bit
   */
  uint8_t badDllpMsk;
  /** 
   * @brief [rw] Bad TLP Mask
   *
   * Field size: 1 bit
   */
  uint8_t badTlpMsk;
  /** 
   * @brief [rw] Receiver Error Mask
   *
   * Field size: 1 bit
   */
  uint8_t rcvrErrMsk;
} pcieCorErrMaskReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_cap_ext_structures
 * @brief Specification of the Advanced capabilities and control Register
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * @{
 */
typedef struct pcieAccrReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /** 
   * @brief [rw] ECRC Check Enable 
   *
   * Field size:  1bit
   */
  uint8_t chkEn;
  /** 
   * @brief [rw] ECRC Check Capable 
   *
   * Field size:  1 bit
   */
  uint8_t chkCap;
  /** 
   * @brief [rw] ECRC Generation Enable  
   *
   * Field size:  1 bit
   */
  uint8_t genEn;
  /** 
   * @brief [rw] ECRC Generation Capability 
   *
   * Field size:  1 bit
   */
  uint8_t genCap;
  /** 
   * @brief [rw] First Error Pointer
   *
   * The First Error Pointer is a field that identifies the bit position 
   * of the first error reported in the @ref pcieUncErrReg_s 
   *
   * Field size:  5 bits
   */
  uint8_t erPtr;
} pcieAccrReg_t;
/* @} */

/** 
 * @ingroup pcielld_reg_cfg_cap_ext_structures 
 * @brief Specification of the Header Log registers
 *
 * These registers may be used for both endpoint and root complex modes.
 *
 * There are 4 Header Log registers
 *
 * @{
 */
typedef struct pcieHdrLogReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */  
  /** 
   * @brief [ro] DWORD of header for a detected error
   *
   * Field size: 32 bits
   */
  uint32_t hdrDW;
} pcieHdrLogReg_t;
/* @} */

/** 
 * @ingroup pcielld_reg_cfg_cap_ext_structures 
 * @brief Specification of the Root Error Command register
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * @{
 */
typedef struct pcieRootErrCmdReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */  
  /** 
   * @brief [rw] Fatal Error Reporting Enable.
   *
   * Field size:  1 bit
   */
  uint8_t ferrRptEn;
  /** 
   * @brief [rw] Nonfatal Error Reporting Enable. 
   *
   * Field size:  1 bit
   */
  uint8_t nferrRptEn;
  /** 
   * @brief [rw] Correctable Error Reporting Enable.
   *
   * Field size:  1 bit
   */
  uint8_t cerrRptEn;
} pcieRootErrCmdReg_t;
/* @} */

/** 
 * @ingroup pcielld_reg_cfg_cap_ext_structures 
 * @brief Specification of the Root Error Status register
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * @{
 */
typedef struct pcieRootErrStReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */  
  /** 
   * @brief [rw] AER Interrupt Message Number.
   *
   * Field size:  5 bits
   */
  uint8_t aerIntMsg;
  /** 
   * @brief [rw] Fatal Error Messages Received.
   *
   * Write 1 to clear
   *
   * Field size:  1 bit
   */
  uint8_t ferrRcv;
  /** 
   * @brief [rw] Non-Fatal Error Messages Received.
   *
   * Write 1 to clear
   *
   * Field size:  1 bit
   */
  uint8_t nfErr;
  /** 
   * @brief [rw] First Uncorrectable Fatal Received.
   *
   * Write 1 to clear
   *
   * Field size:  1 bit
   */
  uint8_t uncorFatal;
  /** 
   * @brief [rw] Multiple Uncorrectable Error (ERR_FATAL/NONFATAL) Received.
   *
   * Write 1 to clear
   *
   * Field size:  1 bit
   */
  uint8_t multFnf;
  /** 
   * @brief [rw] Uncorrectable Error (ERR_FATAL/NONFATAL) Received.
   *
   * Write 1 to clear
   *
   * Field size:  1 bit
   */
  uint8_t errFnf;
  /** 
   * @brief [rw] Multiple Correctable Error (ERR_COR) Received.
   *
   * Write 1 to clear
   *
   * Field size:  1 bit
   */
  uint8_t multCor;
  /** 
   * @brief [rw] Correctable Error (ERR_COR) Received.
   *
   * Write 1 to clear
   *
   * Field size:  1 bit
   */
  uint8_t corrErr;
} pcieRootErrStReg_t;
/* @} */

/** 
 * @ingroup pcielld_reg_cfg_cap_ext_structures 
 * @brief Specification of the Error Source Identification register
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * @{
 */
typedef struct pcieErrSrcIDReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */  
  /** 
   * @brief [ro] Fatal or Non-Fatal error source identification
   *
   * Field size:  16 bits
   */
  uint16_t fnfSrcID;
  /** 
   * @brief [ro] Correctable error source identification
   *
   * Field size:  16 bits
   */
  uint16_t corrSrcID;
} pcieErrSrcIDReg_t;
/* @} */

/*****************************************************************************
 **********  PCIe LOCAL/REMOTE PORT LOGIC REGISTERS **************************
 ****************************************************************************/
/** 
 * @ingroup pcielld_reg_cfg_pl_structures 
 * @brief Specification of the Ack Latency Time and Replay Timer register
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * @{
 */
typedef struct pciePlAckTimerReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */  
  /** 
   * @brief [rw] Replay Time Limit. 
   *
   * The replay timer expires when it reaches this limit. 
   * Write sticky, a reset will not clear this field
   *
   * Field size:  16 bits
   */
  uint16_t rplyLmt;
  /** 
   * @brief [rw] Round Trip Latency Time Limit. 
   *
   * The Ack/Nak latency timer expires when it reaches this limit.
   * Write sticky, a reset will not clear this field
   *
   * Field size:  16 bits
   */
  uint16_t rndTrpLmt;
} pciePlAckTimerReg_t;
/* @} */

/** 
 * @ingroup pcielld_reg_cfg_pl_structures 
 * @brief Specification of the Other Message register
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * @{
 */
typedef struct pciePlOMsgReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */  
  /** 
   * @brief [rw] Other Message Register
   *
   * It can be used to send a specific PCI Express message in which
   * case this register is programmed with the payload and bit 
   * @ref pcieLnkCtrlReg_s::msgReq set to transmit the message.
   *
   * Field size:  32 bits
   */
  uint32_t oMsg;
} pciePlOMsgReg_t;
/* @} */

/** 
 * @ingroup pcielld_reg_cfg_pl_structures 
 * @brief Specification of the Port Force Link register
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * @{
 */
typedef struct pciePlForceLinkReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */  
  /** 
   * @brief [rw] Low Power Entrance Count
   *
   * Field size:  8 bits
   */
  uint8_t lpeCnt;
  /** 
   * @brief [rw] Link State. 
   *
   * The link state that the PCIe will be forced to when 
   * @ref forceLink field is set.  See @ref pcieLtssmState_e
   * for LTSSM states encoded values.
   *
   * Field size:  6 bits
   */
  uint8_t lnkState;
  /** 
   * @brief [rw] Force Link. 
   *
   * Forces the link to the state specified by the @ref lnkState field. 
   * The Force Link pulse will trigger link re-negotiation.
   *
   * Field size:  1 bit
   */
  uint8_t forceLink;
  /** 
   * @brief [rw] Link Number. Not used for EP.
   *
   * Field size:  8 bits
   */
  uint8_t linkNum;
} pciePlForceLinkReg_t;
/* @} */

/** 
 * @ingroup pcielld_reg_cfg_pl_structures 
 * @brief Specification of the Ack Frequency register
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * @{
 */
typedef struct pcieAckFreqReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */  
  /** 
   * @brief [rw] Allow ASPM L1 without partner going to L0s.
   *
   * Set to allow entering ASPM L1 even when link partner did not 
   * go to L0s. When cleared, the ASPM L1 state is entered only after idle 
   * period during which both RX and TX are in L0s.
   *
   * Field size:  1 bit
   */
  uint8_t aspmL1;
  /** 
   * @brief [rw] L1 entrance latency. 
   *
   * The latency is set to 2^@ref l1EntryLatency microseconds with
   * the max being 64 microseconds.
   * <TABLE>
   * <TR><TH>@ref l1EntryLatency</TH><TH>latency in µs</TH></TR>
   * <TR><TD>0</TD>                  <TD>1µs</TD></TR>
   * <TR><TD>1</TD>                  <TD>2µs</TD></TR>
   * <TR><TD>2</TD>                  <TD>4µs</TD></TR>
   * <TR><TD>3</TD>                  <TD>8µs</TD></TR>
   * <TR><TD>4</TD>                  <TD>16µs</TD></TR>
   * <TR><TD>5</TD>                  <TD>32µs</TD></TR>
   * <TR><TD>6</TD>                  <TD>64µs</TD></TR>
   * <TR><TD>7</TD>                  <TD>64µs</TD></TR>
   * </TABLE>
   *
   * Field size:  3 bits
   */
  uint8_t l1EntryLatency;
  /** 
   * @brief [rw] L0s entrance latency. 
   *
   * The latency is set to @ref l0sEntryLatency + 1 microseconds.
   * Maximum is 7 microseconds.
   * 
   * <TABLE>
   * <TR><TH>@ref l0sEntryLatency</TH><TH>latency in µs</TH></TR>
   * <TR><TD>0</TD>                   <TD>1µs</TD></TR>
   * <TR><TD>1</TD>                   <TD>2µs</TD></TR>
   * <TR><TD>2</TD>                   <TD>3µs</TD></TR>
   * <TR><TD>3</TD>                   <TD>4µs</TD></TR>
   * <TR><TD>4</TD>                   <TD>5µs</TD></TR>
   * <TR><TD>5</TD>                   <TD>6µs</TD></TR>
   * <TR><TD>6</TD>                   <TD>7µs</TD></TR>
   * <TR><TD>7</TD>                   <TD>7µs</TD></TR>
   * </TABLE>
   *
   * Field size:  3 bits
   */
  uint8_t l0sEntryLatency;
  /** 
   * @brief [rw] Number of fast training sequences for common clock
   *
   * Number of fast training sequences when common clock is used 
   * and when transitioning from L0s to L0.
   *
   * Field size:  8 bits
   */
  uint8_t commNFts;
  /** 
   * @brief [rw] Number of fast training sequences to be transmitted 
   *
   * Number of fast training sequences to be transmitted 
   * when transitioning from L0s to L0. Value of 0 is not supported.
   *
   * Field size:  8 bits
   */
  uint8_t nFts;
  /** 
   * @brief [rw] Ack Frequency. 
   *
   * Default is to wait until 255 Ack DLLPs are pending before it is sent.
   *
   * Field size:  8 bits
   */
  uint8_t ackFreq;
} pcieAckFreqReg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_pl_structures
 * @brief Specification of the Port Link Control Register
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * @{
 */
typedef struct pcieLnkCtrlReg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /** 
   * @brief [rw] Link Mode 
   *
   * <TABLE>
   * <TR><TH>@ref lnkMode</TH><TH># of lanes</TH></TR>
   * <TR><TD>0x1</TD>         <TD>1</TD></TR>
   * <TR><TD>0x3</TD>         <TD>2</TD></TR>
   * <TR><TD>0x7</TD>         <TD>4</TD></TR>
   * <TR><TD>0xf</TD>         <TD>8</TD></TR>
   * <TR><TD>0x1f</TD>        <TD>16</TD></TR>
   * <TR><TD>0x3f</TD>        <TD>32</TD></TR>
   * <TR><TD>others</TD>      <TD>reserved</TD></TR>
   * </TABLE>
   *
   * Field size: 6 bits
   */
  uint8_t lnkMode;
  /** 
   * @brief [rw] Link Rate 
   *
   * For 2.5 GT/s it is 0x1. This register does not affect any functionality.
   *
   * Field size: 4 bits
   */
  uint8_t lnkRate;
  /** 
   * @brief [rw] Fast link mode 
   *
   * Set all internal timers to fast mode for simulation purposes.
   *
   * Field size: 1 bit
   */
  uint8_t fLnkMode;
  /** 
   * @brief [rw] DLL link enable 
   *
   * DLL Link Enable. Enable link initialization.
   *
   * Field size: 1 bit
   */
  uint8_t dllEn;
  /** 
   * @brief [rw] Reset Assert 
   *
   * Triggers a recovery and forces the LTSSM to the Hot Reset state.
   * Downstream ports (RC ports) only.
   * 
   * Field size: 1 bit
   */
  uint8_t rstAsrt;
  /** 
   * @brief [rw] Loopback Enable 
   *
   * Field size: 1 bit
   */
  uint8_t lpbkEn;
  /** 
   * @brief [rw] Scramble Disable  
   *
   * Field size: 1 bit
   */
  uint8_t scrmDis;
  /** 
   * @brief [rw] Other Message Request 
   *
   * Set to transmit the message contained in @ref pciePlOMsgReg_s
   * 
   * Field size: 1 bit
   */
  uint8_t msgReq;
} pcieLnkCtrlReg_t;
/* @} */

/** 
 * @ingroup pcielld_reg_cfg_pl_structures 
 * @brief Specification of the Lane Skew register
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * @{
 */
typedef struct pcieLaneSkewReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */  
  /** 
   * @brief [rw] Set to Disable Lane to Lane Deskew.
   *
   * Field size: 1 bit
   */
  uint8_t l2Deskew;
  /** 
   * @brief [rw] Set to disable Ack and Nak DLLP transmission. 
   *
   * Field size: 1 bit
   */
  uint8_t ackDisable;
  /** 
   * @brief [rw] Set to disable transmission of Flow Control DLLPs.
   *
   * Field size: 1 bit
   */
  uint8_t fcDisable;
  /** 
   * @brief [rw] Insert Lane Skew for Transmit. 
   *
   * The value is in units of one symbol time. Thus a value 0x02 will
   * force a skew of two symbol times for that lane. Max allowed is 
   * 5 symbol times. This 24 bit field is used for programming skew 
   * for eight lanes with three bits per lane.
   *
   * Field size: 24 bits
   */
  uint32_t laneSkew;
} pcieLaneSkewReg_t;
/* @} */

/** 
 * @ingroup pcielld_reg_cfg_pl_structures 
 * @brief Specification of the Symbol Number register
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * @{
 */
typedef struct pcieSymNumReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */  
  /** 
   * @brief [rw] Configuration requests targeted at function numbers above this 
   * value will result in UR response.
   *
   * Field size: 3 bits
   */
  uint8_t maxFunc;
  /** 
   * @brief [rw] Timer Modifier for Flow Control Watchdog Timer. 
   *
   * Increases the timer value for Flow Control watchdog timer in 
   * increments of 16 clock cycles.
   *
   * Field size: 5 bits
   */
  uint8_t fcWatchTimer;
  /** 
   * @brief [rw] Timer Modifier for Ack/Nak Latency Timer. 
   *
   * Increases the timer value for the Ack/Nak latency timer in
   * increments of 64 clock periods.
   *
   * Field size: 5 bits
   */
  uint8_t ackLatencyTimer;
  /** 
   * @brief [rw] Timer for replaying TLPs in increments of 64 clock cycles.
   *
   * Field size: 5 bits
   */
  uint8_t replayTimer;
  /** 
   * @brief [rw] Number of SKP Symbols.
   *
   * Field size: 3 bits
   */
  uint8_t skpCount;
  /** 
   * @brief [rw] Number of TS2 Symbols. 
   *
   * This field does not affect any functionality.
   *
   * Field size: 4 bits
   */
  uint8_t numTs2Symbols;
  /** 
   * @brief [rw] Number of TS Symbols. 
   *
   * Set the number of TS identifier symbols that are sent in TS1 and TS2 
   * ordered sets.
   *
   * Field size: 4 bits
   */
  uint8_t tsCount;
} pcieSymNumReg_t;
/* @} */

/** 
 * @ingroup pcielld_reg_cfg_pl_structures 
 * @brief Specification of the Symbol Timer and Filter Mask register
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * @{
 */
typedef struct pcieSymTimerFltMaskReg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */  
  /** 
   * @brief [rw] 1 = Allow CFG transaction being received on RC.
   *
   * Field size: 1 bit
   */
  uint8_t f1CfgDrop;
  /** 
   * @brief [rw] 1 = Allow IO transaction being received on RC.
   *
   * Field size: 1 bit
   */
  uint8_t f1IoDrop;
  /** 
   * @brief [rw] 1 = Allow MSG transaction being received on RC.
   *
   * Field size: 1 bit
   */
  uint8_t f1MsgDrop;
  /** 
   * @brief [rw] 1 = Allow completion TLPs with ECRC errors to be passed up. 
   *
   * Field size: 1 bit
   */
  uint8_t f1CplEcrcDrop;
  /** 
   * @brief [rw] 1 = Allow TLPs with ECRC errors to be passed up.
   *
   * Field size: 1 bit
   */
  uint8_t f1EcrcDrop;
  /** 
   * @brief [rw] 1 = Mask length match for received completion TLPs.
   *
   * Field size: 1 bit
   */
  uint8_t f1CplLenTest;
  /** 
   * @brief [rw] 1 = Mask attribute match on received completion TLPs. 
   *
   * Field size: 1 bit
   */
  uint8_t f1CplAttrTest;
  /** 
   * @brief [rw] 1 = Mask traffic class match on received completion TLPs.
   *
   * Field size: 1 bit
   */
  uint8_t f1CplTcTest;
  /** 
   * @brief [rw] 1 = Mask function match for received completion TLPs.
   *
   * Field size: 1 bit
   */
  uint8_t f1CplFuncTest;
  /** 
   * @brief [rw] 1 = Mask request ID match for received completion TLPs. 
   *
   * Field size: 1 bit
   */
  uint8_t f1CplReqIDTest;
  /** 
   * @brief [rw] 1 = Mask tag error rules for received completion TLPs.
   *
   * Field size: 1 bit
   */
  uint8_t f1CplTagErrTest;
  /** 
   * @brief [rw] 1 = Treat locked read TLPs as supported for EP, UR for RC.
   *
   * Field size: 1 bit
   */
  uint8_t f1LockedRdAsUr;
  /** 
   * @brief [rw] 1 = Treat type 1 CFG TLPs as supported for EP and UR for RC.
   *
   * Field size: 1 bit
   */
  uint8_t f1Cfg1ReAsUs;
  /** 
   * @brief [rw] 1 = Treat out-of-BAR TLPs as supported requests.
   *
   * Field size: 1 bit
   */
  uint8_t f1UrOutOfBar;
  /** 
   * @brief [rw] 1 = Treat poisoned TLPs as supported requests.
   *
   * Field size: 1 bit
   */
  uint8_t f1UrPoison;
  /** 
   * @brief [rw] 1 = Treat function mismatched TLPs as supported requests.
   *
   * Field size: 1 bit
   */
  uint8_t f1UrFunMismatch;
  /** 
   * @brief [rw] 1 = Disable Flow Control watchdog timer.
   *
   * Field size: 1 bit
   */
  uint8_t fcWdogDisable;
  /** 
   * @brief [rw] Wait time between SKP ordered sets
   *
   * Number of symbol times to wait between transmitting SKP 
   * ordered sets. For example, for a setting of 1536 decimal, 
   * the wait will be for 1537 symbol times.
   *
   * Field size: 11 bits
   */
  uint16_t skpValue;
} pcieSymTimerFltMaskReg_t;
/* @} */

/** 
 * @ingroup pcielld_reg_cfg_pl_structures 
 * @brief Specification of the Filter Mask 2 register
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * @{
 */
typedef struct pcieFltMask2Reg_s {
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */  
  /** 
   * @brief [rw] 1 = Enable the filter to handle flush request.
   *
   * Field size: 1 bit
   */
  uint8_t flushReq;
  /** 
   * @brief [rw] 1 = Disable DLLP abort for unexpected CPL.
   *
   * Field size: 1 bit
   */
  uint8_t dllpAbort;
  /** 
   * @brief [rw] 1 = Disable dropping of Vendor MSG Type 1.
   *
   * Field size: 1 bit
   */
  uint8_t vmsg1Drop;
  /** 
   * @brief [rw] 1 = Disable dropping of Vendor MSG Type 0 with UR error reporting.
   *
   * Field size: 1 bit
   */
  uint8_t vmsg0Drop;
} pcieFltMask2Reg_t;
/* @} */

/**
 * @ingroup pcielld_reg_cfg_pl_structures
 * @brief Specification of the Debug0 Register
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * @{
 */
typedef struct pcieDebug0Reg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /** 
   * @brief [ro] Link control bits advertised by link partner. 
   *
   * Field size: 4 bits
   */
  uint8_t tsLnkCtrl;
  /** 
   * @brief [ro] Currently receiving k237 (PAD) in place of lane number. 
   *
   * Field size: 1 bit
   */
  uint8_t tsLaneK237;
  /** 
   * @brief [ro] Currently receiving k237 (PAD) in place of link number. 
   *
   * Field size: 1 bit
   */
  uint8_t tsLinkK237;
  /** 
   * @brief [ro] Receiver is receiving logical idle 
   *
   * Field size: 1 bit
   */
  uint8_t rcvdIdle0;
  /** 
   * @brief [ro] 2nd symbol is also idle 
   *
   * Field size: 1 bit
   */
  uint8_t rcvdIdle1;
  /** 
   * @brief [ro] Pipe TX data 
   *
   * Field size: 16 bits
   */
  uint16_t pipeTxData;
  /** 
   * @brief [ro] Pipe transmit K indication
   *
   * Field size: 2 bits
   */
  uint8_t pipeTxDataK;
  /** 
   * @brief [ro] A skip ordered set has been transmitted. 
   *
   * Field size: 1 bit
   */
  uint8_t skipTx;
  /**
   * @brief [ro] LTSSM current state @ref pcieLtssmState_e
   *
   * Field size: 5 bits
   */
  uint8_t ltssmState;
} pcieDebug0Reg_t;
/* @} */

/** 
 * @ingroup pcielld_reg_cfg_pl_structures 
 * @brief Specification of the Debug 1 Register 
 *                                                   
 * This register may be used for both endpoint and root complex modes.
 *                                                    
 * @{                                                  
 */                                                     
typedef struct pcieDebug1Reg_s {                            
  uint32_t raw; /**<  [ro] Raw image of register on read; actual value on write */  
  /** 
   * @brief [rw] Scrambling disabled for the link.            
   *            
   * Field size: 1 bit
   */
  uint8_t scramblerDisable;
  /** 
   * @brief [rw] LTSSM in DISABLE state. Link inoperable.           
   *            
   * Field size: 1 bit
   */
  uint8_t linkDisable;
  /** 
   * @brief [rw] LTSSM performing link training.             
   *            
   * Field size: 1 bit
   */
  uint8_t linkInTraining;
  /** 
   * @brief [rw] LTSSM testing for polarity reversal.            
   *            
   * Field size: 1 bit
   */
  uint8_t rcvrRevrsPolEn;
  /** 
   * @brief [rw] LTSSM-negotiated link reset.              
   *            
   * Field size: 1 bit
   */
  uint8_t trainingRstN;
  /** 
   * @brief [rw] PIPE receiver detect/loopback request.             
   *            
   * Field size: 1 bit
   */
  uint8_t pipeTxdetectrxLb;
  /** 
   * @brief [rw] PIPE transmit electrical idle request.            
   *            
   * Field size: 1 bit
   */
  uint8_t pipeTxelecidle;
  /** 
   * @brief [rw] PIPE transmit compliance request.             
   *            
   * Field size: 1 bit
   */
  uint8_t pipeTxcompliance;
  /** 
   * @brief [rw] Application request to initiate training reset.           
   *            
   * Field size: 1 bit
   */
  uint8_t appInitRst;
  /** 
   * @brief [rw] Link number advertised/confirmed by link partner.           
   *            
   * Field size: 8 bits
   */
  uint8_t rmlhTsLinkNum;
  /** 
   * @brief [rw] LTSSM reports PHY link up.            
   *            
   * Field size: 1 bit
   */
  uint8_t xmlhLinkUp;
  /** 
   * @brief [rw] Receiver reports skip reception.             
   *            
   * Field size: 1 bit
   */
  uint8_t rmlhInskipRcv;
  /** 
   * @brief [rw] TS1 training sequence received (pulse).            
   *            
   * Field size: 1 bit
   */
  uint8_t rmlhTs1Rcvd;
  /** 
   * @brief [rw] TS2 training sequence received (pulse).            
   *            
   * Field size: 1 bit
   */
  uint8_t rmlhTs2Rcvd;
  /** 
   * @brief [rw] Receiver detected lane reversal.             
   *            
   * Field size: 1 bit
   */
  uint8_t rmlhRcvdLaneRev;
} pcieDebug1Reg_t;
/* @} */


/**
 * @ingroup pcielld_reg_cfg_pl_structures
 * @brief Specification of the Gen2 Register
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * @{
 */
typedef struct pcieGen2Reg_s {
  uint32_t raw; /**< @brief [ro] Raw image of register on read; actual value on write */
  /** 
   * @brief [rw] Set de-emphasis level for upstream (EP) ports 
   *
   * Field size: 1 bit
   */
  uint8_t deemph;
  /**
   * @brief [rw] Configure TX compliance receive bit.
   *
   * Field size: 1 bit
   */
  uint8_t txCmpl;
  /**
   * @brief [rw] Configure PHY TX Swing
   *
   * 0 = Low Swing\n
   * 1 = Full Swing
   *
   * Field size: 1 bit
   */
  uint8_t txSwing;
  /**
   * @brief [rw] direct speed change
   *
   * 0 = Indicates to the LTSSM not to initiate a speed change to Gen2 
   * after the link is initialized at Gen1 speed.\n
   * 1 = Indicates to the LTSSM to initiate a speed change to Gen2 
   * after the link is initialized at Gen1 speed.
   *
   * Field size: 1 bit
   */
  uint8_t dirSpd;
  /**
   * @brief [rw]  Lane enable. 1h=x1, 2h=x2. Other values reserved.
   *
   * Field size: 9 bits
   */
  uint16_t lnEn;
  /**
   * @brief [rw] number of fast training sequences
   *
   * Field size: 8 bit
   */
  uint8_t numFts;
} pcieGen2Reg_t;
/* @} */


/**
 * @ingroup pcielld_api_structures
 * @brief Specification all registers
 *
 * This structure allows one or more registers to be read or written
 * through a single call. 
 *
 * The user populates one or more pointers to structures.  All structures
 * that are non-NULL are read or written.
 *
 * Once the pointers are populated, use @ref Pcie_readRegs and/or 
 * @ref Pcie_writeRegs to perform the actual register accesses
 *
 */
typedef struct pcieRegisters_s {

  /*****************************************************************************************
  *Application Registers
  *****************************************************************************************/
  pciePidReg_t                      *pid;                     /**< @brief PID */
  pcieCmdStatusReg_t                *cmdStatus;               /**< @brief Command Status*/
  pcieCfgTransReg_t                 *cfgTrans;                /**< @brief Config Transaction*/
  pcieIoBaseReg_t                   *ioBase;                  /**< @brief IO TLP base*/
  pcieTlpCfgReg_t                   *tlpCfg;                  /**< @brief TLP Config*/
  pcieRstCmdReg_t                   *rstCmd;                  /**< @brief Reset Command*/
  pciePmCmdReg_t                    *pmCmd;                   /**< @brief Power Management Command*/
  pciePmCfgReg_t                    *pmCfg;                   /**< @brief Power Management Config*/
  pcieActStatusReg_t                *actStatus;               /**< @brief Activity Status */
  pcieObSizeReg_t                   *obSize;                  /**< @brief Outbound Translation region size*/
  pcieDiagCtrlReg_t                 *diagCtrl;                /**< @brief Diagnostic Control */
  pcieEndianReg_t                   *endian;                  /**< @brief Endian Register*/
  pciePriorityReg_t                 *priority;                /**< @brief Transaction Priority Register */
  pcieIrqEOIReg_t                   *irqEOI;                  /**< @brief End of Interrupt Register */
  pcieMsiIrqReg_t                   *msiIrq;                  /**< @brief MSI Interrupt IRQ Register*/
  pcieEpIrqSetReg_t                 *epIrqSet;                /**< @brief Endpoint Interrupt Request Set Register*/
  pcieEpIrqClrReg_t                 *epIrqClr;                /**< @brief Endpoint Interrupt Request clear Register*/
  pcieEpIrqStatusReg_t              *epIrqStatus;             /**< @brief Endpoint Interrupt status Register*/
  pcieGenPurposeReg_t               *genPurpose[4];           /**< @brief General Purpose Registers */
  pcieMsiIrqStatusRawReg_t          *msiIrqStatusRaw[8];      /**< @brief MSI Raw Interrupt Status Register*/
  pcieMsiIrqStatusReg_t             *msiIrqStatus[8];         /**< @brief MSI Interrupt Enabled Status Register*/
  pcieMsiIrqEnableSetReg_t          *msiIrqEnableSet[8];      /**< @brief MSI Interrupt Enable Set Register*/
  pcieMsiIrqEnableClrReg_t          *msiIrqEnableClr[8];      /**< @brief MSI Interrupt Enable Clear Register*/
  pcieLegacyIrqStatusRawReg_t       *legacyIrqStatusRaw[4];   /**< @brief Raw Interrupt Status Register*/
  pcieLegacyIrqStatusReg_t          *legacyIrqStatus[4];      /**< @brief Interrupt Enabled Status Register*/
  pcieLegacyIrqEnableSetReg_t       *legacyIrqEnableSet[4];   /**< @brief Interrupt Enable Set Register*/
  pcieLegacyIrqEnableClrReg_t       *legacyIrqEnableClr[4];   /**< @brief Interrupt Enable Clear Register*/
  pcieErrIrqStatusRawReg_t          *errIrqStatusRaw;         /**< @brief Raw Interrupt Status Register*/
  pcieErrIrqStatusReg_t             *errIrqStatus;            /**< @brief Interrupt Enabled Status Register*/
  pcieErrIrqEnableSetReg_t          *errIrqEnableSet;         /**< @brief Interrupt Enable Set Register*/
  pcieErrIrqEnableClrReg_t          *errIrqEnableClr;         /**< @brief Interrupt Enable Clear Register*/
  pciePmRstIrqStatusRawReg_t        *pmRstIrqStatusRaw;       /**< @brief Power Management and Reset Raw Interrupt Status Register*/
  pciePmRstIrqStatusReg_t           *pmRstIrqStatus;          /**< @brief Power Management and Reset Interrupt Enabled Status Register*/
  pciePmRstIrqEnableSetReg_t        *pmRstIrqEnableSet;       /**< @brief Power Management and Reset Interrupt Enable Set Register*/
  pciePmRstIrqEnableClrReg_t        *pmRstIrqEnableClr;       /**< @brief Power Management and Reset Interrupt Enable Clear Register*/
  pcieObOffsetLoReg_t               *obOffsetLo[8];           /**< @brief Outbound Translation region offset Low*/
  pcieObOffsetHiReg_t               *obOffsetHi[8];           /**< @brief Outbound Translation region offset High*/
  pcieIbBarReg_t                    *ibBar[4];                /**< @brief Inbound Translation BAR*/
  pcieIbStartLoReg_t                *ibStartLo[4];            /**< @brief Inbound Translation start Low*/
  pcieIbStartHiReg_t                *ibStartHi[4];            /**< @brief Inbound Translation start High*/
  pcieIbOffsetReg_t                 *ibOffset[4];             /**< @brief Inbound Translation offset*/
  pciePcsCfg0Reg_t                  *pcsCfg0;                 /**< @brief PCS Configuration 0 Register */
  pciePcsCfg1Reg_t                  *pcsCfg1;                 /**< @brief PCS Configuration 1 Register */
  pciePcsStatusReg_t                *pcsStatus;               /**< @brief PCS Status Register */
  pcieSerdesCfg0Reg_t               *serdesCfg0;              /**< @brief SERDES config 0 Register*/
  pcieSerdesCfg1Reg_t               *serdesCfg1;              /**< @brief SERDES config 1 Register*/
  
  /*****************************************************************************************
  *Configuration Registers
  *****************************************************************************************/

  /*Type 0, Type1 Common Registers*/
  pcieVndDevIdReg_t                 *vndDevId;                /**< @brief Vendor and device ID*/
  pcieStatusCmdReg_t                *statusCmd;               /**< @brief Status Command*/
  pcieRevIdReg_t                    *revId;                   /**< @brief Class code and Revision ID*/

  /*Type 0 Registers*/
  pcieBistReg_t                     *bist;                    /**< @brief Bist Header*/
  pcieType0BarIdx_t                 *type0BarIdx;             /**< @brief Type 0 (EP) BAR register*/
  pcieType0Bar32bitIdx_t            *type0Bar32bitIdx;        /**< @brief Type 0 BAR 32bits register*/
  pcieSubIdReg_t                    *subId;                   /**< @brief Subsystem ID*/
  pcieExpRomReg_t                   *expRom;                  /**< @brief Expansion ROM base addr*/
  pcieCapPtrReg_t                   *capPtr;                  /**< @brief Capabilities Pointer*/
  pcieIntPinReg_t                   *intPin;                  /**< @brief Interrupt Pin*/

  /*Type 1 Registers*/
  pcieType1BistHeaderReg_t          *type1BistHeader;         /**< @brief Bist Header, Latency Timer, Cache Line */
  pcieType1BarIdx_t                 *type1BarIdx;             /**< @brief Type 1 (RC) BAR register*/
  pcieType1Bar32bitIdx_t            *type1Bar32bitIdx;        /**< @brief Type 1 BAR 32bits register*/
  pcieType1BusNumReg_t              *type1BusNum;             /**< @brief Latency Timer and Bus Number */
  pcieType1SecStatReg_t             *type1SecStat;            /**< @brief Secondary Status and IO space */
  pcieType1MemspaceReg_t            *type1Memspace;           /**< @brief Memory Limit*/
  pciePrefMemReg_t                  *prefMem;                 /**< @brief Prefetch Memory Limit and Base*/
  pciePrefBaseUpperReg_t            *prefBaseUpper;           /**< @brief Prefetch Memory Base Upper*/
  pciePrefLimitUpperReg_t           *prefLimitUpper;          /**< @brief Prefetch Memory Limit Upper*/
  pcieType1IOSpaceReg_t             *type1IOSpace;            /**< @brief IO Base and Limit Upper 16 bits */
  pcieType1CapPtrReg_t              *type1CapPtr;             /**< @brief Capabilities pointer */
  pcieType1ExpnsnRomReg_t           *type1ExpnsnRom;          /**< @brief Expansion ROM base addr */
  pcieType1BridgeIntReg_t           *type1BridgeInt;          /**< @brief Bridge Control and Interrupt Pointer */

  /* Power Management Capabilities Registers */
  pciePMCapReg_t                    *pmCap;                   /**< @brief Power Management Capabilities */
  pciePMCapCtlStatReg_t             *pmCapCtlStat;            /**< @brief Power Management Control and Status */

  /*MSI Capabilities Registers*/
  pcieMsiCapReg_t                   *msiCap;                  /**< @brief MSI Capabilities */
  pcieMsiLo32Reg_t                  *msiLo32;                 /**< @brief MSI Lower 32 bits */
  pcieMsiUp32Reg_t                  *msiUp32;                 /**< @brief MSI Upper 32 bits */
  pcieMsiDataReg_t                  *msiData;                 /**< @brief MSI Data */

  /*Capabilities Registers*/
  pciePciesCapReg_t                 *pciesCap;                /**< @brief PCI Express Capabilities Register*/
  pcieDeviceCapReg_t                *deviceCap;               /**< @brief Device Capabilities Register*/
  pcieDevStatCtrlReg_t              *devStatCtrl;             /**< @brief Device Status and Control*/
  pcieLinkCapReg_t                  *linkCap;                 /**< @brief Link Capabilities Register*/
  pcieLinkStatCtrlReg_t             *linkStatCtrl;            /**< @brief Link Status and Control Register*/
  pcieSlotCapReg_t                  *slotCap;                 /**< @brief Slot Capabilities Register */
  pcieSlotStatCtrlReg_t             *slotStatCtrl;            /**< @brief Slot Status and Control Register */
  pcieRootCtrlCapReg_t              *rootCtrlCap;             /**< @brief Root Control and Capabilities Register */
  pcieRootStatusReg_t               *rootStatus;              /**< @brief Root Status and Control Register */
  pcieDevCap2Reg_t                  *devCap2;                 /**< @brief Device Capabilities 2 Register*/
  pcieDevStatCtrl2Reg_t             *devStatCtrl2;            /**< @brief Device Status and Control 2 Register*/
  pcieLinkCtrl2Reg_t                *linkCtrl2;               /**< @brief Link Control 2 Register*/

  /*Capabilities Extended Registers*/
  pcieExtCapReg_t                   *extCap;                  /**< @brief Extended Capabilties Header */
  pcieUncErrReg_t                   *uncErr;                  /**< @brief Uncorrectable Error Status */
  pcieUncErrMaskReg_t               *uncErrMask;              /**< @brief Uncorrectable Error Mask */
  pcieUncErrSvrtyReg_t              *uncErrSvrty;             /**< @brief Uncorrectable Error Severity */
  pcieCorErrReg_t                   *corErr;                  /**< @brief Correctable Error Status */
  pcieCorErrMaskReg_t               *corErrMask;              /**< @brief Correctable Error Mask */
  pcieAccrReg_t                     *accr;                    /**< @brief Advanced Capabilities and Control*/
  pcieHdrLogReg_t                   *hdrLog[4];               /**< @brief Header Log Registers */
  pcieRootErrCmdReg_t               *rootErrCmd;              /**< @brief Root Error Command */
  pcieRootErrStReg_t                *rootErrSt;               /**< @brief Root Error Status */
  pcieErrSrcIDReg_t                 *errSrcID;                /**< @brief Error Source Identification */

  /*Port Logic Registers*/
  pciePlAckTimerReg_t               *plAckTimer;              /**< @brief Ack Latency Time and Replay Timer */
  pciePlOMsgReg_t                   *plOMsg;                  /**< @brief Other Message */
  pciePlForceLinkReg_t              *plForceLink;             /**< @brief Port Force Link */
  pcieAckFreqReg_t                  *ackFreq;                 /**< @brief Ack Frequency */
  pcieLnkCtrlReg_t                  *lnkCtrl;                 /**< @brief Port Link Control*/
  pcieLaneSkewReg_t                 *laneSkew;                /**< @brief Lane Skew */
  pcieSymNumReg_t                   *symNum;                  /**< @brief Symbol Number */
  pcieSymTimerFltMaskReg_t          *symTimerFltMask;         /**< @brief Symbol Timer and Filter Mask */
  pcieFltMask2Reg_t                 *fltMask2;                /**< @brief Filter Mask 2 */
  pcieDebug0Reg_t                   *debug0;                  /**< @brief Debug 0*/
  pcieDebug1Reg_t                   *debug1;                  /**< @brief Debug 1 Register*/
  pcieGen2Reg_t                     *gen2;                    /**< @brief Gen2 */
} pcieRegisters_t;

/*****************************************************************************
 **********  Configuration Structures **********************
 ****************************************************************************/

/**
 * @ingroup pcielld_api_structures
 * @brief Specification of pcieIbTransCfg 
 *
 * The pcieIbTransCfg is used to configure the Inbound Translation Registers
 */
typedef struct pcieIbTransCfg_s {
  /** 
   * @brief Inbound Translation BAR match 
   */
  uint8_t    ibBar;   
  /** 
   * @brief Low Inbound Start address (32bits) 
   */
  uint32_t   ibStartAddrLo;
  /** 
   * @brief High Inbound Start address (32bits) 
   */
  uint32_t   ibStartAddrHi;
  /** 
   * @brief Inbound Translation Address Offset (32bits) 
   */
  uint32_t   ibOffsetAddr; 
  /** 
   * @brief Identifies the translation region (0-3) 
   */
  uint8_t    region;    
} pcieIbTransCfg_t;

/**
 * @ingroup pcielld_api_structures
 * @brief Specification of pcieBarCfg
 *
 * The pcieBarCfg is used to configure a 32bits BAR Register
 * or the lower 32bits of a 64bits BAR register. \n
 * This should NOT be used to configure BAR masks.\n
 * This should NOT be used to configure the Upper 32bits of
 * a 64bits BAR register.
 */
typedef struct pcieBarCfg_s {
  /** 
   * @brief Local or remote peripheral 
   */
  pcieLocation_e location; 
  /** 
   * @brief PCIe mode 
   */
  pcieMode_e mode; 
  /** 
   * @brief Base Address (32bits) 
   */
  uint32_t base;
  /**
   * @brief Prefetch
   */
  pcieBarPref_e prefetch;
  /**
   * @brief Type
   */
  pcieBarType_e type;
  /**
   * @brief Memory Space
   */
  pcieBarMem_e memSpace;
  /** 
   * @brief BAR index 
   */
  uint8_t idx;    
} pcieBarCfg_t;

/**
 * @ingroup pcielld_api_structures
 * @brief Specification of Pcie_DeviceCfgBaseAddr 
 *
 * The Pcie_DeviceCfg is used to specify device level configuration
 * to the LLD.
 */
typedef struct
{
  void              *cfgBase;
  void              *dataBase;
  volatile uint32_t *pcieSSModeAddr; /**< address of PCIESSMODE register */
  uint32_t           pcieSSModeMask;  /**< mask for PCIESSMODE field in @ref pcieSSModeAddr */
  uint32_t           pcieSSModeShift; /**< shift for PCIESSMODE field in @ref pcieSSModeAddr */
} Pcie_DeviceCfgBaseAddr;

/**
 * @ingroup pcielld_api_structures
 * @brief Specification of Pcie_DeviceCfg 
 *
 * The Pcie_DeviceCfg is used to specify device level configuration
 * to the LLD.
 */
#define pcie_MAX_PERIPHS 4 /**< Maximum peripherals (base addresses) supported by LLD */
typedef struct
{
  Pcie_DeviceCfgBaseAddr bases[pcie_MAX_PERIPHS]; /**< base addreses */
} Pcie_DeviceCfg;

/**
 * @ingroup pcielld_api_structures
 * @brief Specification of Pcie_InitCfg 
 *
 * The Pcie_InitCfg is used to specify per-core
 * configuration to the LLD.  It is used with @ref Pcie_init ()
 * once per core.
 */
typedef struct
{
  Pcie_DeviceCfg dev; /**< Device Configuration */
} Pcie_InitCfg;

/**
 * @ingroup pcielld_api_structures
 * @brief Specification of Pcie_Handle 
 *
 * The Pcie_Handle is used to identify a PCIE LLD instance
 */
typedef void *Pcie_Handle;

/**
 *  @ingroup pcielld_api_functions
 *  @brief  Pcie_init sets device configuration
 *
 *  @details This function binds a device configuration to
 *           the LLD.  This must be done before calling
 *           the other APIs.
 *
 *           Calling init is nondestrictive, ie it can be
 *           done more than once without side effects (assuming
 *           same argument is passed each time).
 *
 *  @pre     No assumptions
 *
 *  @retval  pcieRet_e status
 *
 *  @post    pcieLObj.device gets set to argument
 */
pcieRet_e Pcie_init
(
  const Pcie_InitCfg *cfg /**< [in] configuration */
);

/**
 *  @ingroup pcielld_api_functions
 *  @brief  Pcie_open creates/opens a PCIe instance
 *
 *  @details This function creates a handle.  The peripheral itself
 *           is not modified.  More than one handle to the same
 *           PCIe peripheral can exist at the same time.
 *
 *  @pre     pHandle != NULL
 *  @pre     *pHandle == NULL
 *
 *  @retval  pcieRet_e status
 *
 *  @post    *pHandle == valid handle
 */
pcieRet_e Pcie_open 
(
  int            deviceNum,  /**< [in] PCIe device number (0,1,...) */
  Pcie_Handle   *pHandle     /**< [out] Resulting instance handle */
);

/**
 *  @ingroup pcielld_api_functions
 *  @brief Pcie_close Closes (frees) the driver handle.
 *
 *  @details The handle is released.  The peripheral itself is not
 *           modified.
 *
 *  @pre     pHandle != NULL
 *  @pre     *pHandle == valid handle
 *
 *  @retval  pcieRet_e status
 *
 *  @post    *pHandle == NULL
 */
pcieRet_e Pcie_close 
(
  Pcie_Handle *pHandle /**< [in] The PCIE LLD instance indentifier */
);

/**
 *  @ingroup pcielld_api_functions
 *  @brief Pcie_readRegs Performs a register read
 *
 *  @details Reads one or more of the device registers
 *
 *  Each non-NULL register pointer in readRegs will be read and 
 *  broken into its fields.
 * 
 *  Some registers have multiple instances (e.g. BAR registers, there is
 *  BAR0, BAR1, etc). In these cases an "index" is used to select which
 *  instance of the register will be accessed (e.g. use index 0 to access BAR0 and so on).
 *
 *  Registers that need an "index" input
 *  can only be read one index at a time. Also, "index" must be set 
 *  by the code before issuing a read request. 
 *
 *  It is the users responsibility to ensure that no other tasks
 *  or cores will modify the registers while they are read,
 *  or betwen the time the registers are read and they are 
 *  later written back.
 *
 *
 *  Since the peripheral is shared across the device, and even
 *  between peripherals, it is not expected to be dynamically
 *  reprogramed (such as between thread or task switches).  It
 *  should only be reprogrammed at startup or when changing
 *  applications.  Therefore, there is a single-entry API instead
 *  of a set of inlines since it is not time-critical code.
 *
 *  @retval  pcieRet_e status
 */
pcieRet_e Pcie_readRegs 
(
  Pcie_Handle      handle,   /**< [in] The PCIE LLD instance identifier */
  pcieLocation_e   location, /**< [in] Local or remote peripheral */
  pcieRegisters_t *readRegs  /**< [in/out] List of registers to read */
);

/**
 *  @ingroup pcielld_api_functions
 *  @brief Pcie_writeRegs Performs a configuration write
 *
 *  @details Writes one or more of the device registers.
 *
 *  Each non-NULL register pointer in writeRegs will be writen.
 * 
 *  Some registers have multiple instances (e.g. BAR registers, there is
 *  BAR0, BAR1, etc). In these cases an "index" is used to select which
 *  instance of the register will be accessed (e.g. use index 0 to access BAR0 and so on).
 *
 *  Registers that need an "index" input can only be written 
 *  one index at a time.
 *
 *  It is the users responsibility to ensure that no other tasks
 *  or cores will modify the registers while they are read,
 *  or betwen the time the registers are read and they are 
 *  later written back.
 *
 *  The user will typically use @ref Pcie_readRegs to read the current 
 *  values in the registers, modify them in the local copies, then
 *  write back using @ref Pcie_writeRegs.
 *
 *  On exit, the actual written values are returned in each register's
 *  reg->raw.
 *
 *  Since the peripheral is shared across the device, and even
 *  between peripherals, it is not expected to be dynamically
 *  reprogramed (such as between thread or task switches).  It
 *  should only be reprogrammed at startup or when changing
 *  applications.  Therefore, there is a single-entry API instead
 *  of a set of inlines since it is not time-critical code.
 *
 *  @retval  pcieRet_e status
 */
pcieRet_e Pcie_writeRegs 
(
  Pcie_Handle      handle,   /**< [in] The PCIE LLD instance identifier */
  pcieLocation_e   location, /**< [in] Local or remote peripheral */
  pcieRegisters_t *writeRegs /**< [in] List of registers to write */
);

/**
 *  @ingroup pcielld_api_functions
 *  @brief  Pcie_setInterfaceMode sets the PCI mode for specified interface
 *
 *  Note: if the PCIESSMODE field is in a register protected by a kicker
 *  mechanism, unlock it before calling this function.  It is not 
 *  multicore safe to repeatedly unlock/lock the kicker.
 *
 *  @retval  pcieRet_e status
 */
pcieRet_e Pcie_setInterfaceMode
(
  Pcie_Handle handle, /**< [in] specified interface */
  pcieMode_e mode     /**< [in] PCIE Mode */
);

/**
 *  @ingroup pcielld_api_functions
 *  @brief  Pcie_setMode sets the PCI mode for all configured interfaces
 *
 *  Note: if the PCIESSMODE field is in a register protected by a kicker
 *  mechanism, unlock it before calling this function.  It is not 
 *  multicore safe to repeatedly unlock/lock the kicker.
 *
 *  @retval  pcieRet_e status
 */
pcieRet_e Pcie_setMode
(
  pcieMode_e mode   /**< [in] PCIE Mode */
);

/**
 *  @ingroup pcielld_api_functions
 *  @brief  Pcie_getMemSpaceRange Returns the PCIe Internal Address 
 *  Range for the device's internal address range 1, which is
 *  the Memory Space. 
 *
 *  @retval  pcieRet_e status
 */
pcieRet_e Pcie_getMemSpaceRange 
(
  Pcie_Handle  handle,   /**< [in] The PCIE LLD instance identifier */
  void         **base,   /**< [out] The memory space base address */
  uint32_t      *size    /**< [out] Total size of the memory space [bytes] */
);


/**
 *  @ingroup pcielld_api_functions
 *  @brief  Pcie_cfgObOffset Configures the Outbound Offset registers 
 *          for outbound address translation
 *
 *  @retval  pcieRet_e status
 */
pcieRet_e Pcie_cfgObOffset 
(
  Pcie_Handle      handle,   /**< [in] The PCIE LLD instance identifier */
  uint32_t         obAddrLo, /**< [in] Low  Outbound address offset (32bits) */
  uint32_t         obAddrHi, /**< [in] High Outbound address offset (32bits) */
  uint8_t          region    /**< [in] Identifies the Outbound region (0-7) */
);

/**
 *  @ingroup pcielld_api_functions
 *  @brief  Pcie_cfgIbTrans Configures the Inbound Address Translation registers.
 *
 *  @retval  pcieRet_e status
 */
pcieRet_e Pcie_cfgIbTrans 
(
  Pcie_Handle      handle,   /**< [in] The PCIE LLD instance identifier */
  pcieIbTransCfg_t *ibCfg    /**< [in] Inbound Address Translation Configuration parameters */
);

/**
 *  @ingroup pcielld_api_functions
 *  @brief  Pcie_cfgBar is used to configure a 32bits BAR Register
 *
 *  A BAR register can represent any of the following:\n
 *  (a) a 32bit BAR\n 
 *  (b) the lower 32bits of a 64bits BAR\n
 *  (c) the upper 32bits of a 64bits BAR\n 
 *  (d) a BAR mask\n
 *
 *  BAR registers can always be accessed using @ref Pcie_readRegs 
 *  and/or @ref Pcie_writeRegs.
 *
 *  @ref Pcie_cfgBar is used to configure a 32bits BAR Register or the lower
 *  32bits of a 64bits BAR register. That is, (a) and (b) above.
 *
 *  @ref Pcie_cfgBar should NOT be used to configure the Upper 32bits of a 64bits BAR register (c).\n
 *  @ref Pcie_cfgBar should NOT be used to configure BAR masks (d). \n\n
 *  In order to access BAR masks or Upper 32bits BAR, use @ref Pcie_readRegs and/or 
 *  @ref Pcie_writeRegs to perform the actual 32bits register accesses, using
 *  @ref pcieType0Bar32bitIdx_t (for a End point BAR) or @ref pcieType1Bar32bitIdx_t (for a Root Complex BAR).
 *
 *
 *  @retval  pcieRet_e status
 */
pcieRet_e Pcie_cfgBar 
(
  Pcie_Handle      handle,   /**< [in] The PCIE LLD instance identifier */
  pcieBarCfg_t    *barCfg    /**< [in] BAR configuration parameters */
);


/**
 *  @ingroup pcielld_api_functions
 *  @brief  Pcie_getVersion returns the PCIE LLD version information
 *
 *  @details This function is used to get the version information of the PCIE LLD in 0xAABBCCDD format.
 *           where Arch (AA); API Changes (BB); Major (CC); Minor (DD)
 *
 *  @retval                32-bit version information
 */
uint32_t Pcie_getVersion (void);


/**
 *  @ingroup pcielld_api_functions
 *  @brief  Pcie_getVersionStr returns the PCIE LLD version string
 *
 *  @details This function is used to get the version string of the PCIE LLD.
 *
 *  @retval                Version string
 */
const char* Pcie_getVersionStr (void);

#ifdef __cplusplus
}
#endif
  
#endif  /* _PCIE_H */

/* Nothing past this point */

