/**
 * @file w5500_reg.h
 * @brief W5500 Ethernet Controller Register Map
 */

#ifndef W5500_REG_H
#define W5500_REG_H

/* ============================================================================ */
/* W5500 Block Select Bits (BSB) Definition */
/* ============================================================================ */
#define W5500_BSB_COMMON_REG            0x00U    /*!< Common Register Block */
#define W5500_BSB_SOCKET0_REG           0x01U    /*!< Socket 0 Register Block */
#define W5500_BSB_SOCKET1_REG           0x05U    /*!< Socket 1 Register Block */
#define W5500_BSB_SOCKET2_REG           0x09U    /*!< Socket 2 Register Block */
#define W5500_BSB_SOCKET3_REG           0x0DU    /*!< Socket 3 Register Block */
#define W5500_BSB_SOCKET4_REG           0x11U    /*!< Socket 4 Register Block */
#define W5500_BSB_SOCKET5_REG           0x15U    /*!< Socket 5 Register Block */
#define W5500_BSB_SOCKET6_REG           0x19U    /*!< Socket 6 Register Block */
#define W5500_BSB_SOCKET7_REG           0x1DU    /*!< Socket 7 Register Block */

#define W5500_BSB_SOCKET0_TX            0x02U    /*!< Socket 0 TX Buffer */
#define W5500_BSB_SOCKET1_TX            0x06U    /*!< Socket 1 TX Buffer */
#define W5500_BSB_SOCKET2_TX            0x0AU    /*!< Socket 2 TX Buffer */
#define W5500_BSB_SOCKET3_TX            0x0EU    /*!< Socket 3 TX Buffer */
#define W5500_BSB_SOCKET4_TX            0x12U    /*!< Socket 4 TX Buffer */
#define W5500_BSB_SOCKET5_TX            0x16U    /*!< Socket 5 TX Buffer */
#define W5500_BSB_SOCKET6_TX            0x1AU    /*!< Socket 6 TX Buffer */
#define W5500_BSB_SOCKET7_TX            0x1EU    /*!< Socket 7 TX Buffer */

#define W5500_BSB_SOCKET0_RX            0x03U    /*!< Socket 0 RX Buffer */
#define W5500_BSB_SOCKET1_RX            0x07U    /*!< Socket 1 RX Buffer */
#define W5500_BSB_SOCKET2_RX            0x0BU    /*!< Socket 2 RX Buffer */
#define W5500_BSB_SOCKET3_RX            0x0FU    /*!< Socket 3 RX Buffer */
#define W5500_BSB_SOCKET4_RX            0x13U    /*!< Socket 4 RX Buffer */
#define W5500_BSB_SOCKET5_RX            0x17U    /*!< Socket 5 RX Buffer */
#define W5500_BSB_SOCKET6_RX            0x1BU    /*!< Socket 6 RX Buffer */
#define W5500_BSB_SOCKET7_RX            0x1FU    /*!< Socket 7 RX Buffer */

/* ============================================================================ */
/* Common Registers - POPRAWIONE ADRESY */
/* ============================================================================ */
#define W5500_MR                        0x0000U  /*!< Mode Register */
#define W5500_GAR0                      0x0001U  /*!< Gateway Address Register 0 */
#define W5500_GAR1                      0x0002U  /*!< Gateway Address Register 1 */
#define W5500_GAR2                      0x0003U  /*!< Gateway Address Register 2 */
#define W5500_GAR3                      0x0004U  /*!< Gateway Address Register 3 */

/* SUBNET MASK - BRAKOWAŁO */
#define W5500_SUBR0                     0x0005U  /*!< Subnet Mask Register 0 */
#define W5500_SUBR1                     0x0006U  /*!< Subnet Mask Register 1 */
#define W5500_SUBR2                     0x0007U  /*!< Subnet Mask Register 2 */
#define W5500_SUBR3                     0x0008U  /*!< Subnet Mask Register 3 */

/* SOURCE MAC - POPRAWIONE ADRESY */
#define W5500_SHAR0                     0x0009U  /*!< Source Hardware Address Register 0 */
#define W5500_SHAR1                     0x000AU  /*!< Source Hardware Address Register 1 */
#define W5500_SHAR2                     0x000BU  /*!< Source Hardware Address Register 2 */
#define W5500_SHAR3                     0x000CU  /*!< Source Hardware Address Register 3 */
#define W5500_SHAR4                     0x000DU  /*!< Source Hardware Address Register 4 */
#define W5500_SHAR5                     0x000EU  /*!< Source Hardware Address Register 5 */

/* SOURCE IP */
#define W5500_SIPR0                     0x000FU  /*!< Source IP Address Register 0 */
#define W5500_SIPR1                     0x0010U  /*!< Source IP Address Register 1 */
#define W5500_SIPR2                     0x0011U  /*!< Source IP Address Register 2 */
#define W5500_SIPR3                     0x0012U  /*!< Source IP Address Register 3 */

/* INTERRUPT REGISTERS */
#define W5500_INTLEVEL0                 0x0013U  /*!< Interrupt Low Level Timer Register 0 */
#define W5500_INTLEVEL1                 0x0014U  /*!< Interrupt Low Level Timer Register 1 */
#define W5500_IR                        0x0015U  /*!< Interrupt Register */
#define W5500_IMR                       0x0016U  /*!< Interrupt Mask Register */
#define W5500_SIR                       0x0017U  /*!< Socket Interrupt Register */
#define W5500_SIMR                      0x0018U  /*!< Socket Interrupt Mask Register */

/* RETRY REGISTERS */
#define W5500_RTR0                      0x0019U  /*!< Retry Time Register 0 */
#define W5500_RTR1                      0x001AU  /*!< Retry Time Register 1 */
#define W5500_RCR                       0x001BU  /*!< Retry Count Register */

/* PPPoE REGISTERS */
#define W5500_PTIMER                    0x001CU  /*!< PPP LCP Request Timer Register */
#define W5500_PMAGIC                    0x001DU  /*!< PPP LCP Magic Number Register */

/* PPPoE HARDWARE ADDRESS - POPRAWIONE ADRESY */
#define W5500_PHAR0                     0x001EU  /*!< PPPoE Hardware Address Register 0 */
#define W5500_PHAR1                     0x001FU  /*!< PPPoE Hardware Address Register 1 */
#define W5500_PHAR2                     0x0020U  /*!< PPPoE Hardware Address Register 2 */
#define W5500_PHAR3                     0x0021U  /*!< PPPoE Hardware Address Register 3 */
#define W5500_PHAR4                     0x0022U  /*!< PPPoE Hardware Address Register 4 */
#define W5500_PHAR5                     0x0023U  /*!< PPPoE Hardware Address Register 5 */

/* PPPoE SESSION */
#define W5500_PSID0                     0x0024U  /*!< PPPoE Session ID Register 0 */
#define W5500_PSID1                     0x0025U  /*!< PPPoE Session ID Register 1 */

/* PPPoE MTU */
#define W5500_PMRU0                     0x0026U  /*!< PPPoE Maximum Receive Unit Register 0 */
#define W5500_PMRU1                     0x0027U  /*!< PPPoE Maximum Receive Unit Register 1 */

/* UNREACHABLE DESTINATION - BRAKOWAŁO */
#define W5500_UIPR0                     0x0028U  /*!< Unreachable IP Address Register 0 */
#define W5500_UIPR1                     0x0029U  /*!< Unreachable IP Address Register 1 */
#define W5500_UIPR2                     0x002AU  /*!< Unreachable IP Address Register 2 */
#define W5500_UIPR3                     0x002BU  /*!< Unreachable IP Address Register 3 */

#define W5500_UPORTR0                   0x002CU  /*!< Unreachable Port Register 0 */
#define W5500_UPORTR1                   0x002DU  /*!< Unreachable Port Register 1 */

/* PHY CONFIGURATION - BRAKOWAŁO */
#define W5500_PHYCFGR                   0x002EU  /*!< PHY Configuration Register */

/* CHIP VERSION - BRAKOWAŁO */
#define W5500_VERSIONR                  0x0039U  /*!< Chip Version Register */

/* ============================================================================ */
/* Socket Registers - SPRAWDZONE */
/* ============================================================================ */
#define W5500_Sn_MR                     0x0000U  /*!< Socket Mode Register */
#define W5500_Sn_CR                     0x0001U  /*!< Socket Command Register */
#define W5500_Sn_IR                     0x0002U  /*!< Socket Interrupt Register */
#define W5500_Sn_SR                     0x0003U  /*!< Socket Status Register */
#define W5500_Sn_PORT0                  0x0004U  /*!< Socket Source Port Register 0 */
#define W5500_Sn_PORT1                  0x0005U  /*!< Socket Source Port Register 1 */
#define W5500_Sn_DHAR0                  0x0006U  /*!< Socket Destination Hardware Address 0 */
#define W5500_Sn_DHAR1                  0x0007U  /*!< Socket Destination Hardware Address 1 */
#define W5500_Sn_DHAR2                  0x0008U  /*!< Socket Destination Hardware Address 2 */
#define W5500_Sn_DHAR3                  0x0009U  /*!< Socket Destination Hardware Address 3 */
#define W5500_Sn_DHAR4                  0x000AU  /*!< Socket Destination Hardware Address 4 */
#define W5500_Sn_DHAR5                  0x000BU  /*!< Socket Destination Hardware Address 5 */
#define W5500_Sn_DIPR0                  0x000CU  /*!< Socket Destination IP Address 0 */
#define W5500_Sn_DIPR1                  0x000DU  /*!< Socket Destination IP Address 1 */
#define W5500_Sn_DIPR2                  0x000EU  /*!< Socket Destination IP Address 2 */
#define W5500_Sn_DIPR3                  0x000FU  /*!< Socket Destination IP Address 3 */
#define W5500_Sn_DPORT0                 0x0010U  /*!< Socket Destination Port 0 */
#define W5500_Sn_DPORT1                 0x0011U  /*!< Socket Destination Port 1 */
#define W5500_Sn_MSSR0                  0x0012U  /*!< Socket Maximum Segment Size 0 */
#define W5500_Sn_MSSR1                  0x0013U  /*!< Socket Maximum Segment Size 1 */
#define W5500_Sn_TOS                    0x0015U  /*!< Socket Type of Service */
#define W5500_Sn_TTL                    0x0016U  /*!< Socket Time to Live */
#define W5500_Sn_RXBUFSIZE              0x001EU  /*!< Socket RX Buffer Size */
#define W5500_Sn_TXBUFSIZE              0x001FU  /*!< Socket TX Buffer Size */
#define W5500_Sn_TX_FSR0                0x0020U  /*!< Socket TX Free Size 0 */
#define W5500_Sn_TX_FSR1                0x0021U  /*!< Socket TX Free Size 1 */
#define W5500_Sn_TX_RD0                 0x0022U  /*!< Socket TX Read Pointer 0 */
#define W5500_Sn_TX_RD1                 0x0023U  /*!< Socket TX Read Pointer 1 */
#define W5500_Sn_TX_WR0                 0x0024U  /*!< Socket TX Write Pointer 0 */
#define W5500_Sn_TX_WR1                 0x0025U  /*!< Socket TX Write Pointer 1 */
#define W5500_Sn_RX_RSR0                0x0026U  /*!< Socket RX Received Size 0 */
#define W5500_Sn_RX_RSR1                0x0027U  /*!< Socket RX Received Size 1 */
#define W5500_Sn_RX_RD0                 0x0028U  /*!< Socket RX Read Pointer 0 */
#define W5500_Sn_RX_RD1                 0x0029U  /*!< Socket RX Read Pointer 1 */
#define W5500_Sn_RX_WR0                 0x002AU  /*!< Socket RX Write Pointer 0 */
#define W5500_Sn_RX_WR1                 0x002BU  /*!< Socket RX Write Pointer 1 */
#define W5500_Sn_IMR                    0x002CU  /*!< Socket Interrupt Mask */
#define W5500_Sn_FRAG0                  0x002DU  /*!< Socket Fragment Offset 0 */
#define W5500_Sn_FRAG1                  0x002EU  /*!< Socket Fragment Offset 1 */
#define W5500_Sn_KPALVTR                0x002FU  /*!< Socket Keep Alive Timer (1 bajt!) */

/* ============================================================================ */
/* Bit Definitions */
/* ============================================================================ */

/* Bit definition for W5500_MR register */
#define W5500_MR_RST_Pos                7U
#define W5500_MR_RST_Msk                (0x1UL << W5500_MR_RST_Pos)      /*!< 0x80 */
#define W5500_MR_RST                    W5500_MR_RST_Msk                /*!< Software Reset */

#define W5500_MR_WOL_Pos                5U
#define W5500_MR_WOL_Msk                (0x1UL << W5500_MR_WOL_Pos)      /*!< 0x20 */
#define W5500_MR_WOL                    W5500_MR_WOL_Msk                /*!< Wake on LAN */

#define W5500_MR_PB_Pos                 4U
#define W5500_MR_PB_Msk                 (0x1UL << W5500_MR_PB_Pos)       /*!< 0x10 */
#define W5500_MR_PB                     W5500_MR_PB_Msk                 /*!< Ping Block */

#define W5500_MR_PPPoE_Pos              3U
#define W5500_MR_PPPoE_Msk              (0x1UL << W5500_MR_PPPoE_Pos)    /*!< 0x08 */
#define W5500_MR_PPPoE                  W5500_MR_PPPoE_Msk              /*!< PPPoE Mode */

#define W5500_MR_FARP_Pos               1U
#define W5500_MR_FARP_Msk               (0x1UL << W5500_MR_FARP_Pos)     /*!< 0x02 */
#define W5500_MR_FARP                   W5500_MR_FARP_Msk               /*!< Force ARP */

#define W5500_Sn_MR_MULTI_Pos          7U
#define W5500_Sn_MR_MULTI_Msk          (0x1UL << W5500_Sn_MR_MULTI_Pos) /*!< 0x80 */
#define W5500_Sn_MR_MULTI              W5500_Sn_MR_MULTI_Msk          /*!< Multicasting */

#define W5500_Sn_MR_PROTOCOL_Pos       0U
#define W5500_Sn_MR_PROTOCOL_Msk       (0xFUL << W5500_Sn_MR_PROTOCOL_Pos) /*!< 0x0F */
#define W5500_Sn_MR_PROTOCOL           W5500_Sn_MR_PROTOCOL_Msk       /*!< Protocol */
#define W5500_Sn_MR_PROTOCOL_CLOSED    0x00U                           /*!< Closed */
#define W5500_Sn_MR_PROTOCOL_TCP       0x01U                           /*!< TCP */
#define W5500_Sn_MR_PROTOCOL_UDP       0x02U                           /*!< UDP */
#define W5500_Sn_MR_PROTOCOL_MACRAW    0x04U                           /*!< MAC RAW */


/* Bit definition for W5500_PHYCFGR register */
#define W5500_PHYCFGR_RST_Pos           7U
#define W5500_PHYCFGR_RST_Msk           (0x1UL << W5500_PHYCFGR_RST_Pos) /*!< 0x80 */
#define W5500_PHYCFGR_RST               W5500_PHYCFGR_RST_Msk           /*!< PHY Reset */

#define W5500_PHYCFGR_OPMD_Pos          6U
#define W5500_PHYCFGR_OPMD_Msk          (0x1UL << W5500_PHYCFGR_OPMD_Pos) /*!< 0x40 */
#define W5500_PHYCFGR_OPMD              W5500_PHYCFGR_OPMD_Msk          /*!< Operation Mode */

#define W5500_PHYCFGR_OPMDC_Pos         3U
#define W5500_PHYCFGR_OPMDC_Msk         (0x7UL << W5500_PHYCFGR_OPMDC_Pos) /*!< 0x38 */
#define W5500_PHYCFGR_OPMDC             W5500_PHYCFGR_OPMDC_Msk         /*!< Operation Mode Config */

#define W5500_PHYCFGR_DPX_Pos           2U
#define W5500_PHYCFGR_DPX_Msk           (0x1UL << W5500_PHYCFGR_DPX_Pos) /*!< 0x04 */
#define W5500_PHYCFGR_DPX               W5500_PHYCFGR_DPX_Msk           /*!< Duplex Status */

#define W5500_PHYCFGR_SPD_Pos           1U
#define W5500_PHYCFGR_SPD_Msk           (0x1UL << W5500_PHYCFGR_SPD_Pos) /*!< 0x02 */
#define W5500_PHYCFGR_SPD               W5500_PHYCFGR_SPD_Msk           /*!< Speed Status */

#define W5500_PHYCFGR_LNK_Pos           0U
#define W5500_PHYCFGR_LNK_Msk           (0x1UL << W5500_PHYCFGR_LNK_Pos) /*!< 0x01 */
#define W5500_PHYCFGR_LNK               W5500_PHYCFGR_LNK_Msk           /*!< Link Status */

/* Socket Commands */
#define W5500_Sn_CR_OPEN               0x01U    /*!< Initialize or open socket */
#define W5500_Sn_CR_LISTEN             0x02U    /*!< Wait connection request in TCP mode */
#define W5500_Sn_CR_CONNECT            0x04U    /*!< Send connection request in TCP mode */
#define W5500_Sn_CR_DISCON             0x08U    /*!< Send closing request in TCP mode */
#define W5500_Sn_CR_CLOSE              0x10U    /*!< Close socket */
#define W5500_Sn_CR_SEND               0x20U    /*!< Update TX memory pointer, transmit data */
#define W5500_Sn_CR_SEND_MAC           0x21U    /*!< Send data with MAC address */
#define W5500_Sn_CR_SEND_KEEP          0x22U    /*!< Send keep alive message */
#define W5500_Sn_CR_RECV               0x40U    /*!< Update RX memory pointer, receive data */

/* Socket Status */
#define W5500_Sn_SR_SOCK_CLOSED        0x00U   // Closed
#define W5500_Sn_SR_SOCK_INIT          0x13U   // Initialized
#define W5500_Sn_SR_SOCK_LISTEN        0x14U   // Listen
#define W5500_Sn_SR_SOCK_SYNSENT       0x15U   // SYN sent (BRAKOWAŁO!)
#define W5500_Sn_SR_SOCK_SYNRECV       0x16U   // SYN received (BRAKOWAŁO!)
#define W5500_Sn_SR_SOCK_ESTABLISHED   0x17U   // Success to connect
#define W5500_Sn_SR_SOCK_FINWAIT       0x18U   // Closing state (BRAKOWAŁO!)
#define W5500_Sn_SR_SOCK_CLOSING       0x1AU   // Closing state (BRAKOWAŁO!)
#define W5500_Sn_SR_SOCK_TIMEWAIT      0x1BU   // Time wait (BRAKOWAŁO!)
#define W5500_Sn_SR_SOCK_CLOSEWAIT     0x1CU   // Close wait
#define W5500_Sn_SR_SOCK_LASTACK       0x1DU   // Last ACK (BRAKOWAŁO!)
#define W5500_Sn_SR_SOCK_UDP           0x22U   // UDP socket
#define W5500_Sn_SR_SOCK_MACRAW        0x42U   // MAC raw mode socket

/* SPI Control */
#define W5500_SPI_BSB_Pos               3U
#define W5500_SPI_BSB_Msk               (0x1FUL << W5500_SPI_BSB_Pos)    /*!< 0xF8 */
#define W5500_SPI_BSB                   W5500_SPI_BSB_Msk               /*!< Block Select Bits */

#define W5500_SPI_RWB_Pos               2U
#define W5500_SPI_RWB_Msk               (0x1UL << W5500_SPI_RWB_Pos)     /*!< 0x04 */
#define W5500_SPI_RWB_READ              (0x0UL << W5500_SPI_RWB_Pos)     /*!< Read */
#define W5500_SPI_RWB_WRITE             (0x1UL << W5500_SPI_RWB_Pos)     /*!< Write */

#define W5500_SPI_OM_Pos                0U
#define W5500_SPI_OM_Msk                (0x3UL << W5500_SPI_OM_Pos)      /*!< 0x03 */
#define W5500_SPI_OM_VDM                0x00U                            /*!< Variable Data Mode */
#define W5500_SPI_OM_FDM1               0x01U                            /*!< Fixed Data Mode 1 byte */
#define W5500_SPI_OM_FDM2               0x02U                            /*!< Fixed Data Mode 2 bytes */
#define W5500_SPI_OM_FDM4               0x03U                            /*!< Fixed Data Mode 4 bytes */

// Socket RX/TX Pointers - BRAKOWAŁO
#define W5500_Sn_TXFSR0     0x0020U    // Socket TX Free Size 0
#define W5500_Sn_TXFSR1     0x0021U    // Socket TX Free Size 1
#define W5500_Sn_TXRD0      0x0022U    // Socket TX Read Pointer 0
#define W5500_Sn_TXRD1      0x0023U    // Socket TX Read Pointer 1
#define W5500_Sn_TXWR0      0x0024U    // Socket TX Write Pointer 0
#define W5500_Sn_TXWR1      0x0025U    // Socket TX Write Pointer 1
#define W5500_Sn_RXRSR0     0x0026U    // Socket RX Received Size 0
#define W5500_Sn_RXRSR1     0x0027U    // Socket RX Received Size 1
#define W5500_Sn_RXRD0      0x0028U    // Socket RX Read Pointer 0
#define W5500_Sn_RXRD1      0x0029U    // Socket RX Read Pointer 1
#define W5500_Sn_RXWR0      0x002AU    // Socket RX Write Pointer 0
#define W5500_Sn_RXWR1      0x002BU    // Socket RX Write Pointer 1

#endif /* W5500_REG_H */
