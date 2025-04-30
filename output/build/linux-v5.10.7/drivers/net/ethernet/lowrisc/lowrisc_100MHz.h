// See LICENSE for license details.

#ifndef ETH_HEADER_H
#define ETH_HEADER_H

/* Register offsets for the LowRISC Ethernet Core */

/* Register offsets (in bytes) for the LowRISC Core */
#define TXBUFF_OFFSET       0x1000          /* Transmit Buffer */

#define MACLO_OFFSET        0x0000          /* MAC address low 32-bits */
#define MACHI_OFFSET        0x0004          /* MAC address high 16-bits and MAC ctrl */
#define MDIOCTRL_OFFSET     0x0008          /* MDIO Control Register */
#define TXBUSY_OFFSET       0x000C          /* Tx Busy */
#define TFCS_OFFSET         0x0010          /* Tx frame check sequence register */
#define RFCS_OFFSET         0x0014          /* Rx frame check sequence register(read) and last register(write) */
#define RSR_OFFSET          0x0018
#define SRC_ADDR_OFFSET     0x001c
#define DST_ADDR_OFFSET     0x0020
#define DMA_LENGTH_OFFSET   0x0024
#define SRC_PROTO_OFFSET    0x0028
#define DST_PROTO_OFFSET    0x002C
#define REQ_VALID_OFFSET    0x0044
#define REQ_READY_OFFSET    0x0048
#define RSP_READY_OFFSET    0x004C
#define RSP_VALID_OFFSET    0x0050
#define RX_END_CLR_OFFSET   0x0054
#define VALID_CLR_OFFSET    0x0058

#define AXI                 0x0
#define AXI_STREAM          0x5
/* MAC Ctrl Register (MACHI) Bit Masks */
#define MACHI_MACADDR_MASK    0x0000FFFF     /* MAC high 16-bits mask */
#define MACHI_COOKED_MASK     0x00010000     /* obsolete flag */
#define MACHI_LOOPBACK_MASK   0x00020000     /* Rx loopback packets */
#define MACHI_ALLPKTS_MASK    0x00400000     /* Rx all packets (promiscuous mode) */
#define MACHI_IRQ_EN          0x00800000     /* Rx packet interrupt enable */

/* MDIO Control Register Bit Masks */
#define MDIOCTRL_MDIOCLK_MASK 0x00000001    /* MDIO Clock Mask */
#define MDIOCTRL_MDIOOUT_MASK 0x00000002    /* MDIO Output Mask */
#define MDIOCTRL_MDIOOEN_MASK 0x00000004    /* MDIO Output Enable Mask, 3-state enable, high=input, low=output */
#define MDIOCTRL_MDIORST_MASK 0x00000008    /* MDIO Input Mask */
#define MDIOCTRL_MDIOIN_MASK  0x00000008    /* MDIO Input Mask */

/* Transmit Status Register (TPLR) Bit Masks */
#define TPLR_PACKET_LEN_MASK  0x00000FFF     /* Tx packet length */
#define TX_BUSY_MASK          0x00000001     /* Tx busy mask */

/* Receive Status Register (RSR) */
#define RSR_RECV_DONE_MASK   0x00000001      /* Rx complete */
#define RSR_RECV_IRQ_MASK    0x00000002      /* Rx irq bit */

/* General Ethernet Definitions */
#define HEADER_OFFSET               12      /* Offset to length field */
#define HEADER_SHIFT                16      /* Shift value for length */
#define ARP_PACKET_SIZE             28      /* Max ARP packet size */
#define HEADER_IP_LENGTH_OFFSET     16      /* IP Length Offset */

#endif
