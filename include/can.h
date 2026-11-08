/**
 * \file            can.h
 * \brief           CAN1 transport: the command / telemetry link to the VCU
 * \note            The MCU is a node on the 500 kbit/s scooter CAN bus. It receives drive and
 *                      configuration frames from the VCU and transmits motor-controller status.
 *                      This module owns the peripheral, the acceptance filter, the RX/TX software
 *                      rings and the interrupt handlers; the per-id command semantics live in the
 *                      control module, which drains \ref can_rx_pop and transmits via \ref can_send.
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 *                      The Artery AT32F415 BSP + CMSIS keep their own licenses.
 */
#ifndef CAN_H
#define CAN_H

#include <stdint.h>

/* Software ring depth: 32 slots, index masked with 0x1F (31 usable; one slot stays empty so a
   full ring is distinguishable from an empty one). */
#define MCU_CAN_RING_LEN  32u
#define MCU_CAN_RING_MASK 0x1Fu

/** \brief CAN 2.0 payload limit: a classic data frame carries at most 8 bytes. */
#define MCU_CAN_DLEN      8u

/* ------------------------------------------------------------------------- */
/* Inbound standard-id map (frames the VCU sends to the MCU).                 */
/* Routing only; the payload meaning is decoded in the control module.        */
/* ------------------------------------------------------------------------- */
#define CANID_DRIVE_CMD   0x100U /*!< drive command from the VCU (fastest cadence) */
#define CANID_AUX_CMD     0x20CU /*!< auxiliary command / mode frame               */
#define CANID_CFG_A       0x310U /*!< configuration block A                        */
#define CANID_CFG_B       0x311U /*!< configuration block B                        */
#define CANID_KEEPALIVE   0x423U /*!< keep-alive / comms watchdog refresh          */
/* Custom diagnostic ids: the 0x6xx block is unused on the bus (the drive/ESC/BMS traffic
   sits in 0x0xx-0x5xx, the handshake at 0x7Fx), so these sit clear of a future id and are
   low priority -- they never disturb the real-time drive frames. */
#define CANID_BACKUP_REQ  0x6E0U /*!< read-only backup request (diagnostic)        */
#define CANID_BACKUP_RSP  0x6E1U /*!< read-only backup response (diagnostic)       */

/**
 * \brief           20-byte CAN message record
 * \note            Mirrors the BSP tx/rx message fields in one unpacked record that both rings
 *                      and both interrupt handlers share.
 */
typedef struct {
    uint32_t std_id;            /*!< +0x00 11-bit standard id (used when ide==0) */
    uint32_t ext_id;            /*!< +0x04 29-bit extended id (used when ide==1) */
    uint8_t ide;                /*!< +0x08 0 = standard id, 1 = extended id      */
    uint8_t rtr;                /*!< +0x09 0 = data frame, 1 = remote frame      */
    uint8_t dlc;                /*!< +0x0A data length, 0..8                     */
    uint8_t data[MCU_CAN_DLEN]; /*!< +0x0B payload bytes                         */
    uint8_t fmi;                /*!< +0x13 filter-match index (RX only)          */
} mcu_can_msg_t;

/** \brief Bring up CAN1 (pins, 500 kbit timing, accept-all filter, RX + error interrupts). */
void can_init(void);

/** \brief Service the CAN link: pump the TX ring into the mailboxes (the super-loop body). */
void can_service(void);

/**
 * \brief           Queue a standard-id data frame for transmission
 * \param[in]       std_id: 11-bit standard identifier
 * \param[in]       data: payload bytes (may be NULL when \p len is 0)
 * \param[in]       len: payload length, 0..8
 */
void can_send(uint16_t std_id, const uint8_t* data, uint8_t len);

/**
 * \brief           Dequeue the oldest received frame
 * \param[out]      out: receives the frame when one is available
 * \return          1 when a frame was returned, 0 when the RX ring is empty
 */
uint8_t can_rx_pop(mcu_can_msg_t* out);

#endif /* CAN_H */
