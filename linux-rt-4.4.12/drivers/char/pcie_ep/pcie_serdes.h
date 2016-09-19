#ifndef __PCIE_SERDES_H
#define __PCIF_SERDES_H


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


/** ============================================================================
 * @brief
 *
 *  SERDES LINK RATE speed enumerators */
typedef enum
{
    /** 1.25 GHz */
   CSL_SERDES_LINK_RATE_1p25G        =   0,

    /** 4.9152 GHz */
   CSL_SERDES_LINK_RATE_4p9152G      =   1,

    /** 5 GHz */
   CSL_SERDES_LINK_RATE_5G           =   2,

    /** 6.144 GHz */
   CSL_SERDES_LINK_RATE_6p144G       =   3,

    /** 6.25 GHz */
   CSL_SERDES_LINK_RATE_6p25G        =   4,

   /** 7.3728 GHz */
   CSL_SERDES_LINK_RATE_7p3728G      =   5,

   /** 9.8304 GHz */
   CSL_SERDES_LINK_RATE_9p8304G      =   6,

   /** 10 GHz */
   CSL_SERDES_LINK_RATE_10G          =   7,

   /** 10.3125 GHz */
   CSL_SERDES_LINK_RATE_10p3125G     =   8,

    /** 12.5 GHz */
   CSL_SERDES_LINK_RATE_12p5G        =   9
} CSL_SERDES_LINK_RATE;


/** ============================================================================
 * @brief
 *
 *  SERDES LOOPBACK enumerators */

typedef enum
{
    /** Loopback Enabled */
    CSL_SERDES_LOOPBACK_ENABLED   =  0,

    /** Loopback Disabled */
    CSL_SERDES_LOOPBACK_DISABLED  =  1
} CSL_SERDES_LOOPBACK;


/** ============================================================================
 * @brief
 *
 *  SERDES PLL STATUS enumerators */
typedef enum
{
    /** PLL Not Locked */
    CSL_SERDES_STATUS_PLL_NOT_LOCKED = 0,

    /** PLL Locked */
    CSL_SERDES_STATUS_PLL_LOCKED     = 1
} CSL_SERDES_STATUS;



/** ============================================================================
 * @brief
 *
 *  SERDES INIT RETURN VALUE enumerators */
typedef enum
{
    /** Init Success */
    CSL_SERDES_NO_ERR               = 0,

    /** Invalid Reference Clock */
    CSL_SERDES_INVALID_REF_CLOCK    = 1,

    /** Invalid Lane Rate */
    CSL_SERDES_INVALID_LANE_RATE    = 2
} CSL_SERDES_RESULT;


/** ============================================================================
 * @brief
 *
 *  SERDES LANE CTRL TX/RX RATE enumerators */
typedef enum
{
    /** SERDES Full Rate */
    CSL_SERDES_LANE_FULL_RATE      = 0,

    /** SERDES Half Rate */
    CSL_SERDES_LANE_HALF_RATE      = 1,

    /** SERDES Quarter Rate */
    CSL_SERDES_LANE_QUARTER_RATE   = 2
} CSL_SERDES_LANE_CTRL_RATE;


/** ============================================================================
 * @brief
 *
 *  SERDES LANE CTRL STATUS enumerators */
typedef enum
{
    /** Lane Control Enable Success */
    CSL_SERDES_LANE_ENABLE_NO_ERR           = 0,

    /** Invalid Lane Control Rate */
    CSL_SERDES_LANE_ENABLE_INVALID_RATE     = 1
} CSL_SERDES_LANE_ENABLE_STATUS;


/** ============================================================================
 * @brief
 *
 *  SERDES PHY TYPE enumerators */
typedef enum
{
    /** 10GE SERDES */
    SERDES_10GE = 0,

    /** AIF2 B8 SERDES */
    SERDES_AIF2_B8     = 1,

    /** AIF2 B4 SERDES */
    SERDES_AIF2_B4     = 2,

    /** SRIO SERDES */
    SERDES_SRIO     = 3,

    /** PCIe SERDES */
    SERDES_PCIe     = 4,

    /** Hyperlink SERDES */
    SERDES_HYPERLINK     = 5,

    /** SGMII SERDES */
    SERDES_SGMII     = 6,

    /** DFE SERDES */
    SERDES_DFE       = 7,

    /** IQN SERDES */
    SERDES_IQN       = 8
} csl_serdes_phy_type;

#define CSL_SERDES_MAX_LANES                4
#define CSL_SERDES_MAX_TAPS                 5
#define CSL_SERDES_MAX_COMPARATORS          5
#define CSL_SERDES_TBUS_SIZE                155
#define CSL_SERDES_PHY_A_1LANE_QUAD_MUX     0x4ebe
#define CSL_SERDES_PHY_A_2LANE_NO_MUX       0x4eb8
#define CSL_SERDES_PHY_A_2LANE_QUAD_MUX     0x4ebc
#define CSL_SERDES_PHY_A_4LANE_NO_MUX       0x4eb9
#define CSL_SERDES_PHY_A_4LANE_QUAD_MUX     0x4ebd
#define CSL_SERDES_PHY_B_2LANE_NO_MUX       0x4eba
#define CSL_SERDES_ATT_BOOST_NUM_REPEAT     20
#define CSL_SERDES_ATT_BOOST_REPEAT_MEAN    14

#endif
