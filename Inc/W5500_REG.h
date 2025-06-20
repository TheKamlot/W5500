/**
  ******************************************************************************
  * @file    w5500_reg.h
  * @brief   W5500 Ethernet Controller Register Map
  ******************************************************************************
  */

#ifndef W5500_REG_H
#define W5500_REG_H

/* ================================================================================ */
/* ================                    W5500                     ================ */
/* ================================================================================ */

/**
  * @brief W5500 Block Select Bits (BSB) Definition
  */
#define W5500_BSB_COMMON_REG            0x00U   /*!< Common Register Block */
#define W5500_BSB_SOCKET0_REG           0x01U   /*!< Socket 0 Register Block */
#define W5500_BSB_SOCKET1_REG           0x02U   /*!< Socket 1 Register Block */
#define W5500_BSB_SOCKET2_REG           0x03U   /*!< Socket 2 Register Block */
#define W5500_BSB_SOCKET3_REG           0x04U   /*!< Socket 3 Register Block */
#define W5500_BSB_SOCKET4_REG           0x05U   /*!< Socket 4 Register Block */
#define W5500_BSB_SOCKET5_REG           0x06U   /*!< Socket 5 Register Block */
#define W5500_BSB_SOCKET6_REG           0x07U   /*!< Socket 6 Register Block */
#define W5500_BSB_SOCKET7_REG           0x08U   /*!< Socket 7 Register Block */
#define W5500_BSB_SOCKET0_TX            0x10U   /*!< Socket 0 TX Buffer */
#define W5500_BSB_SOCKET1_TX            0x11U   /*!< Socket 1 TX Buffer */
#define W5500_BSB_SOCKET2_TX            0x12U   /*!< Socket 2 TX Buffer */
#define W5500_BSB_SOCKET3_TX            0x13U   /*!< Socket 3 TX Buffer */
#define W5500_BSB_SOCKET4_TX            0x14U   /*!< Socket 4 TX Buffer */
#define W5500_BSB_SOCKET5_TX            0x15U   /*!< Socket 5 TX Buffer */
#define W5500_BSB_SOCKET6_TX            0x16U   /*!< Socket 6 TX Buffer */
#define W5500_BSB_SOCKET7_TX            0x17U   /*!< Socket 7 TX Buffer */
#define W5500_BSB_SOCKET0_RX            0x18U   /*!< Socket 0 RX Buffer */
#define W5500_BSB_SOCKET1_RX            0x19U   /*!< Socket 1 RX Buffer */
#define W5500_BSB_SOCKET2_RX            0x1AU   /*!< Socket 2 RX Buffer */
#define W5500_BSB_SOCKET3_RX            0x1BU   /*!< Socket 3 RX Buffer */
#define W5500_BSB_SOCKET4_RX            0x1CU   /*!< Socket 4 RX Buffer */
#define W5500_BSB_SOCKET5_RX            0x1DU   /*!< Socket 5 RX Buffer */
#define W5500_BSB_SOCKET6_RX            0x1EU   /*!< Socket 6 RX Buffer */
#define W5500_BSB_SOCKET7_RX            0x1FU   /*!< Socket 7 RX Buffer */

/* ================================================================================ */
/* ================              Common Registers                ================ */
/* ================================================================================ */

#define W5500_MR                        0x0000U /*!< Mode Register */
#define W5500_GAR0                      0x0001U /*!< Gateway Address Register 0 */
#define W5500_GAR1                      0x0002U /*!< Gateway Address Register 1 */
#define W5500_GAR2                      0x0003U /*!< Gateway Address Register 2 */
#define W5500_GAR3                      0x0004U /*!< Gateway Address Register 3 */
#define W5500_SHAR0                     0x0005U /*!< Source Hardware Address Register 0 */
#define W5500_SHAR1                     0x0006U /*!< Source Hardware Address Register 1 */
#define W5500_SHAR2                     0x0007U /*!< Source Hardware Address Register 2 */
#define W5500_SHAR3                     0x0008U /*!< Source Hardware Address Register 3 */
#define W5500_SHAR4                     0x0009U /*!< Source Hardware Address Register 4 */
#define W5500_SHAR5                     0x000AU /*!< Source Hardware Address Register 5 */
#define W5500_SIPR0                     0x000FU /*!< Source IP Address Register 0 */
#define W5500_SIPR1                     0x0010U /*!< Source IP Address Register 1 */
#define W5500_SIPR2                     0x0011U /*!< Source IP Address Register 2 */
#define W5500_SIPR3                     0x0012U /*!< Source IP Address Register 3 */
#define W5500_INTLEVEL0                 0x0013U /*!< Interrupt Low Level Timer Register 0 */
#define W5500_INTLEVEL1                 0x0014U /*!< Interrupt Low Level Timer Register 1 */
#define W5500_IR                        0x0015U /*!< Interrupt Register */
#define W5500_IMR                       0x0016U /*!< Interrupt Mask Register */
#define W5500_SIR                       0x0017U /*!< Socket Interrupt Register */
#define W5500_SIMR                      0x0018U /*!< Socket Interrupt Mask Register */
#define W5500_RTR0                      0x0019U /*!< Retry Time Register 0 */
#define W5500_RTR1                      0x001AU /*!< Retry Time Register 1 */
#define W5500_RCR                       0x001BU /*!< Retry Count Register */
#define W5500_PHAR0                     0x001CU /*!< PPPoE Authentication Hardware Address 0 */
#define W5500_PHAR1                     0x001DU /*!< PPPoE Authentication Hardware Address 1 */
#define W5500_PHAR2                     0x001EU /*!< PPPoE Authentication Hardware Address 2 */
#define W5500_PHAR3                     0x001FU /*!< PPPoE Authentication Hardware Address 3 */
#define W5500_PHAR4                     0x0020U /*!< PPPoE Authentication Hardware Address 4 */
#define W5500_PHAR5                     0x0021U /*!< PPPoE Authentication Hardware Address 5 */

/* ================================================================================ */
/* ================              Socket Registers                ================ */
/* ================================================================================ */

#define W5500_Sn_MR                     0x0000U /*!< Socket Mode Register */
#define W5500_Sn_CR                     0x0001U /*!< Socket Command Register */
#define W5500_Sn_IR                     0x0002U /*!< Socket Interrupt Register */
#define W5500_Sn_SR                     0x0003U /*!< Socket Status Register */
#define W5500_Sn_PORT0                  0x0004U /*!< Socket Source Port Register 0 */
#define W5500_Sn_PORT1                  0x0005U /*!< Socket Source Port Register 1 */
#define W5500_Sn_DHAR0                  0x0006U /*!< Socket Destination Hardware Address 0 */
#define W5500_Sn_DHAR1                  0x0007U /*!< Socket Destination Hardware Address 1 */
#define W5500_Sn_DHAR2                  0x0008U /*!< Socket Destination Hardware Address 2 */
#define W5500_Sn_DHAR3                  0x0009U /*!< Socket Destination Hardware Address 3 */
#define W5500_Sn_DHAR4                  0x000AU /*!< Socket Destination Hardware Address 4 */
#define W5500_Sn_DHAR5                  0x000BU /*!< Socket Destination Hardware Address 5 */
#define W5500_Sn_DIPR0                  0x000CU /*!< Socket Destination IP Address 0 */
#define W5500_Sn_DIPR1                  0x000DU /*!< Socket Destination IP Address 1 */
#define W5500_Sn_DIPR2                  0x000EU /*!< Socket Destination IP Address 2 */
#define W5500_Sn_DIPR3                  0x000FU /*!< Socket Destination IP Address 3 */
#define W5500_Sn_DPORT0                 0x0010U /*!< Socket Destination Port 0 */
#define W5500_Sn_DPORT1                 0x0011U /*!< Socket Destination Port 1 */
#define W5500_Sn_MSSR0                  0x0012U /*!< Socket Maximum Segment Size 0 */
#define W5500_Sn_MSSR1                  0x0013U /*!< Socket Maximum Segment Size 1 */
#define W5500_Sn_TOS                    0x0015U /*!< Socket Type of Service */
#define W5500_Sn_TTL                    0x0016U /*!< Socket Time to Live */
#define W5500_Sn_RXBUF_SIZE             0x001EU /*!< Socket RX Buffer Size */
#define W5500_Sn_TXBUF_SIZE             0x001FU /*!< Socket TX Buffer Size */
#define W5500_Sn_TX_FSR0                0x0020U /*!< Socket TX Free Size 0 */
#define W5500_Sn_TX_FSR1                0x0021U /*!< Socket TX Free Size 1 */
#define W5500_Sn_TX_RD0                 0x0022U /*!< Socket TX Read Pointer 0 */
#define W5500_Sn_TX_RD1                 0x0023U /*!< Socket TX Read Pointer 1 */
#define W5500_Sn_TX_WR0                 0x0024U /*!< Socket TX Write Pointer 0 */
#define W5500_Sn_TX_WR1                 0x0025U /*!< Socket TX Write Pointer 1 */
#define W5500_Sn_RX_RSR0                0x0026U /*!< Socket RX Received Size 0 */
#define W5500_Sn_RX_RSR1                0x0027U /*!< Socket RX Received Size 1 */
#define W5500_Sn_RX_RD0                 0x0028U /*!< Socket RX Read Pointer 0 */
#define W5500_Sn_RX_RD1                 0x0029U /*!< Socket RX Read Pointer 1 */
#define W5500_Sn_RX_WR0                 0x002AU /*!< Socket RX Write Pointer 0 */
#define W5500_Sn_RX_WR1                 0x002BU /*!< Socket RX Write Pointer 1 */
#define W5500_Sn_IMR                    0x002CU /*!< Socket Interrupt Mask */
#define W5500_Sn_FRAG0                  0x002DU /*!< Socket Fragment Offset 0 */
#define W5500_Sn_FRAG1                  0x002EU /*!< Socket Fragment Offset 1 */
#define W5500_Sn_KPALVTR0               0x002FU /*!< Socket Keep Alive Timer 0 */
#define W5500_Sn_KPALVTR1               0x0030U /*!< Socket Keep Alive Timer 1 */

/* ================================================================================ */
/* ================                 Bit Definitions              ================ */
/* ================================================================================ */

/*******************  Bit definition for W5500_MR register  *********************/
#define W5500_MR_RST_Pos                (7U)
#define W5500_MR_RST_Msk                (0x1UL << W5500_MR_RST_Pos)           /*!< 0x80 */
#define W5500_MR_RST                    W5500_MR_RST_Msk                       /*!< Software Reset */
#define W5500_MR_WOL_Pos                (5U)
#define W5500_MR_WOL_Msk                (0x1UL << W5500_MR_WOL_Pos)           /*!< 0x20 */
#define W5500_MR_WOL                    W5500_MR_WOL_Msk                       /*!< Wake on LAN */
#define W5500_MR_PB_Pos                 (4U)
#define W5500_MR_PB_Msk                 (0x1UL << W5500_MR_PB_Pos)            /*!< 0x10 */
#define W5500_MR_PB                     W5500_MR_PB_Msk                        /*!< Ping Block */
#define W5500_MR_PPPoE_Pos              (3U)
#define W5500_MR_PPPoE_Msk              (0x1UL << W5500_MR_PPPoE_Pos)         /*!< 0x08 */
#define W5500_MR_PPPoE                  W5500_MR_PPPoE_Msk                     /*!< PPPoE Mode */
#define W5500_MR_FARP_Pos               (1U)
#define W5500_MR_FARP_Msk               (0x1UL << W5500_MR_FARP_Pos)          /*!< 0x02 */
#define W5500_MR_FARP                   W5500_MR_FARP_Msk                      /*!< Force ARP */

/*******************  Bit definition for W5500_Sn_MR register  ******************/
#define W5500_Sn_MR_MULTI_Pos           (7U)
#define W5500_Sn_MR_MULTI_Msk           (0x1UL << W5500_Sn_MR_MULTI_Pos)      /*!< 0x80 */
#define W5500_Sn_MR_MULTI               W5500_Sn_MR_MULTI_Msk                  /*!< Multicasting */
#define W5500_Sn_MR_BCASTB_Pos          (6U)
#define W5500_Sn_MR_BCASTB_Msk          (0x1UL << W5500_Sn_MR_BCASTB_Pos)     /*!< 0x40 */
#define W5500_Sn_MR_BCASTB              W5500_Sn_MR_BCASTB_Msk                 /*!< Broadcast Block */
#define W5500_Sn_MR_ND_Pos              (5U)
#define W5500_Sn_MR_ND_Msk              (0x1UL << W5500_Sn_MR_ND_Pos)         /*!< 0x20 */
#define W5500_Sn_MR_ND                  W5500_Sn_MR_ND_Msk                     /*!< No Delayed ACK */
#define W5500_Sn_MR_MC_Pos              (5U)
#define W5500_Sn_MR_MC_Msk              (0x1UL << W5500_Sn_MR_MC_Pos)         /*!< 0x20 */
#define W5500_Sn_MR_MC                  W5500_Sn_MR_MC_Msk                     /*!< Multicast */
#define W5500_Sn_MR_MMB_Pos             (5U)
#define W5500_Sn_MR_MMB_Msk             (0x1UL << W5500_Sn_MR_MMB_Pos)        /*!< 0x20 */
#define W5500_Sn_MR_MMB                 W5500_Sn_MR_MMB_Msk                    /*!< MAC Mask Bit */
#define W5500_Sn_MR_UCASTB_Pos          (4U)
#define W5500_Sn_MR_UCASTB_Msk          (0x1UL << W5500_Sn_MR_UCASTB_Pos)     /*!< 0x10 */
#define W5500_Sn_MR_UCASTB              W5500_Sn_MR_UCASTB_Msk                 /*!< Unicast Block */
#define W5500_Sn_MR_MIP6B_Pos           (4U)
#define W5500_Sn_MR_MIP6B_Msk           (0x1UL << W5500_Sn_MR_MIP6B_Pos)      /*!< 0x10 */
#define W5500_Sn_MR_MIP6B               W5500_Sn_MR_MIP6B_Msk                  /*!< IPv6 Packet Block */
#define W5500_Sn_MR_PROTOCOL_Pos        (0U)
#define W5500_Sn_MR_PROTOCOL_Msk        (0xFUL << W5500_Sn_MR_PROTOCOL_Pos)    /*!< 0x0F */
#define W5500_Sn_MR_PROTOCOL            W5500_Sn_MR_PROTOCOL_Msk               /*!< Protocol */
#define W5500_Sn_MR_PROTOCOL_CLOSED     0x00U                                  /*!< Closed */
#define W5500_Sn_MR_PROTOCOL_TCP        0x01U                                  /*!< TCP */
#define W5500_Sn_MR_PROTOCOL_UDP        0x02U                                  /*!< UDP */
#define W5500_Sn_MR_PROTOCOL_IPRAW      0x03U                                  /*!< IP RAW */
#define W5500_Sn_MR_PROTOCOL_MACRAW     0x04U                                  /*!< MAC RAW */
#define W5500_Sn_MR_PROTOCOL_PPPoE      0x05U                                  /*!< PPPoE */

/*******************  Bit definition for W5500_Sn_CR register  ******************/
#define W5500_Sn_CR_OPEN                0x01U   /*!< Initialize or open socket */
#define W5500_Sn_CR_LISTEN              0x02U   /*!< Wait connection request in TCP mode */
#define W5500_Sn_CR_CONNECT             0x04U   /*!< Send connection request in TCP mode */
#define W5500_Sn_CR_DISCON              0x08U   /*!< Send closing request in TCP mode */
#define W5500_Sn_CR_CLOSE               0x10U   /*!< Close socket */
#define W5500_Sn_CR_SEND                0x20U   /*!< Update TX memory pointer, transmit data */
#define W5500_Sn_CR_SEND_MAC            0x21U   /*!< Send data with MAC address */
#define W5500_Sn_CR_SEND_KEEP           0x22U   /*!< Send keep alive message */
#define W5500_Sn_CR_RECV                0x40U   /*!< Update RX memory pointer, receive data */

/*******************  Bit definition for W5500_Sn_SR register  ******************/
#define W5500_Sn_SR_SOCK_CLOSED         0x00U   /*!< Closed */
#define W5500_Sn_SR_SOCK_INIT           0x13U   /*!< Initialized */
#define W5500_Sn_SR_SOCK_LISTEN         0x14U   /*!< Listen */
#define W5500_Sn_SR_SOCK_SYNSENT        0x15U   /*!< Connection state */
#define W5500_Sn_SR_SOCK_SYNRECV        0x16U   /*!< Connection state */
#define W5500_Sn_SR_SOCK_ESTABLISHED    0x17U   /*!< Success to connect */
#define W5500_Sn_SR_SOCK_FIN_WAIT       0x18U   /*!< Closing state */
#define W5500_Sn_SR_SOCK_CLOSING        0x1AU   /*!< Closing state */
#define W5500_Sn_SR_SOCK_TIME_WAIT      0x1BU   /*!< Closing state */
#define W5500_Sn_SR_SOCK_CLOSE_WAIT     0x1CU   /*!< Closing state */
#define W5500_Sn_SR_SOCK_LAST_ACK       0x1DU   /*!< Closing state */
#define W5500_Sn_SR_SOCK_UDP            0x22U   /*!< UDP socket */
#define W5500_Sn_SR_SOCK_IPRAW          0x32U   /*!< IP raw mode socket */
#define W5500_Sn_SR_SOCK_MACRAW         0x42U   /*!< MAC raw mode socket */
#define W5500_Sn_SR_SOCK_PPPoE          0x5FU   /*!< PPPoE mode socket */

/* ================================================================================ */
/* ================                 SPI Control                  ================ */
/* ================================================================================ */

/*******************  SPI Control Byte Definition  *****************************/
#define W5500_SPI_BSB_Pos               (3U)
#define W5500_SPI_BSB_Msk               (0x1FUL << W5500_SPI_BSB_Pos)         /*!< 0xF8 */
#define W5500_SPI_BSB                   W5500_SPI_BSB_Msk                      /*!< Block Select Bits */
#define W5500_SPI_RWB_Pos               (2U)
#define W5500_SPI_RWB_Msk               (0x1UL << W5500_SPI_RWB_Pos)          /*!< 0x04 */
#define W5500_SPI_RWB_READ              (0x0UL << W5500_SPI_RWB_Pos)          /*!< Read */
#define W5500_SPI_RWB_WRITE             (0x1UL << W5500_SPI_RWB_Pos)          /*!< Write */
#define W5500_SPI_OM_Pos                (0U)
#define W5500_SPI_OM_Msk                (0x3UL << W5500_SPI_OM_Pos)           /*!< 0x03 */
#define W5500_SPI_OM_VDM                0x00U                                  /*!< Variable Data Mode */
#define W5500_SPI_OM_FDM1               0x01U                                  /*!< Fixed Data Mode 1 byte */
#define W5500_SPI_OM_FDM2               0x02U                                  /*!< Fixed Data Mode 2 bytes */
#define W5500_SPI_OM_FDM4               0x03U                                  /*!< Fixed Data Mode 4 bytes */

#endif /* W5500_REG_H */

