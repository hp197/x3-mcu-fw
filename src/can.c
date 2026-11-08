/**
 * \file            can.c
 * \brief           CAN1 transport: init, interrupt handlers, TX pump, RX/TX rings
 * \note            The MCU and the VCU share one 500 kbit/s bus. can_init brings up CAN1 with an
 *                      accept-all filter; the RX interrupt copies each received frame into the RX
 *                      ring and the control loop drains it; outbound status frames are queued with
 *                      can_send and pushed into the mailboxes by can_service. The per-id meaning of
 *                      a frame is decoded in the control module, not here.
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 *                      The Artery AT32F415 BSP + CMSIS keep their own licenses.
 */
#include "can.h"

#include <stddef.h>

#include "at32f415.h"

/** \brief One software ring of CAN frames (single-producer / single-consumer). */
typedef struct {
    uint32_t wr;                          /*!< write index (producer) */
    uint32_t rd;                          /*!< read index (consumer)  */
    mcu_can_msg_t slot[MCU_CAN_RING_LEN]; /*!< ring storage           */
} mcu_can_ring_t;

/* RX ring: produced by the CAN1 RX0 interrupt, consumed by can_rx_pop (the control loop).
   TX ring: produced by can_send, consumed by can_service (the mailbox pump). */
static mcu_can_ring_t g_rx_ring;
static mcu_can_ring_t g_tx_ring;

/**
 * \brief           Copy one message record
 * \param[out]      dst: destination record
 * \param[in]       src: source record
 */
static void prv_frame_copy(mcu_can_msg_t* dst, const mcu_can_msg_t* src) {
    uint8_t i;

    dst->std_id = src->std_id;
    dst->ext_id = src->ext_id;
    dst->ide = src->ide;
    dst->rtr = src->rtr;
    dst->dlc = src->dlc;
    dst->fmi = src->fmi;
    for (i = 0; i < MCU_CAN_DLEN; ++i) {
        dst->data[i] = src->data[i];
    }
}

/**
 * \brief           Configure PA11 (CAN1 RX) and PA12 (CAN1 TX) as the CAN alternate function
 */
static void prv_can_gpio_init(void) {
    gpio_init_type gpio_init_struct;

    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    gpio_default_para_init(&gpio_init_struct);

    /* PA11 CAN1 RX: input with pull-up. */
    gpio_init_struct.gpio_pins = GPIO_PINS_11;
    gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
    gpio_init_struct.gpio_pull = GPIO_PULL_UP;
    gpio_init(GPIOA, &gpio_init_struct);

    /* PA12 CAN1 TX: alternate-function push-pull. */
    gpio_init_struct.gpio_pins = GPIO_PINS_12;
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init(GPIOA, &gpio_init_struct);
}

void can_init(void) {
    can_base_type base;
    can_baudrate_type baud;
    can_filter_init_type filter;

    prv_can_gpio_init();
    crm_periph_clock_enable(CRM_CAN1_PERIPH_CLOCK, TRUE);

    /* Normal communication mode; auto-exit doze on; one-shot transmit (no retransmission on a
       failed send, so a stale real-time command is never re-sent); discard-on-overflow; id
       priority for the send order. */
    can_default_para_init(&base);
    base.mode_selection = CAN_MODE_COMMUNICATE;
    base.ttc_enable = FALSE;
    base.aebo_enable = FALSE;
    base.aed_enable = TRUE;
    base.prsf_enable = TRUE;
    base.mdrsel_selection = CAN_DISCARDING_FIRST_RECEIVED;
    base.mmssr_selection = CAN_SENDING_BY_ID;
    can_base_init(CAN1, &base);

    /* 500 kbit/s: APB1 / 9, 16 Tq/bit (1 sync + 11 BS1 + 4 BS2), SJW 2 Tq. */
    baud.baudrate_div = 9;
    baud.rsaw_size = CAN_RSAW_2TQ;
    baud.bts1_size = CAN_BTS1_11TQ;
    baud.bts2_size = CAN_BTS2_4TQ;
    can_baudrate_set(CAN1, &baud);

    /* One 32-bit mask-mode filter, id = mask = 0: accept every frame into FIFO0. No hardware id
       filtering; routing is done in software by the control decoder. */
    filter.filter_activate_enable = TRUE;
    filter.filter_mode = CAN_FILTER_MODE_ID_MASK;
    filter.filter_fifo = CAN_FILTER_FIFO0;
    filter.filter_number = 0;
    filter.filter_bit = CAN_FILTER_32BIT;
    filter.filter_id_high = 0;
    filter.filter_id_low = 0;
    filter.filter_mask_high = 0;
    filter.filter_mask_low = 0;
    can_filter_init(CAN1, &filter);

    /* RX FIFO0 message interrupt + the error / bus-off interrupts. */
    can_interrupt_enable(CAN1, CAN_RF0MIEN_INT | CAN_BOIEN_INT | CAN_ETRIEN_INT | CAN_EOIEN_INT, TRUE);
    nvic_irq_enable(CAN1_SE_IRQn, 0, 0);
    nvic_irq_enable(CAN1_RX0_IRQn, 0, 0);
}

/**
 * \brief           CAN1 RX FIFO0 interrupt: receive one frame and push it onto the RX ring
 */
void CAN1_RX0_IRQHandler(void) {
    if (can_receive_message_pending_get(CAN1, CAN_RX_FIFO0) != 0u) {
        can_rx_message_type rx;
        mcu_can_msg_t frame;
        uint32_t next;
        uint8_t i;

        can_message_receive(CAN1, CAN_RX_FIFO0, &rx);

        frame.std_id = rx.standard_id;
        frame.ext_id = rx.extended_id;
        frame.ide = (uint8_t)rx.id_type;
        frame.rtr = (uint8_t)rx.frame_type;
        frame.dlc = rx.dlc;
        frame.fmi = rx.filter_index;
        for (i = 0; i < MCU_CAN_DLEN; ++i) {
            frame.data[i] = rx.data[i];
        }

        next = (g_rx_ring.wr + 1u) & MCU_CAN_RING_MASK;
        if (next != g_rx_ring.rd) { /* drop the frame when the ring is full */
            prv_frame_copy(&g_rx_ring.slot[g_rx_ring.wr], &frame);
            g_rx_ring.wr = next;
        }
    }
}

/**
 * \brief           CAN1 status-change / error interrupt
 * \note            On an error record, clear the error-status register; always clear the
 *                      error-occur interrupt flag so it does not re-assert. The error counters stay
 *                      readable through the BSP for the control module's bus-health check.
 */
void CAN1_SE_IRQHandler(void) {
    if (CAN1->ests_bit.etr != 0u) {
        CAN1->ests = 0u; /* clear the error record */
    }
    CAN1->msts = CAN_MSTS_EOIF_VAL; /* write 1 to clear the error-occur interrupt flag */
}

void can_service(void) {
    while (g_tx_ring.rd != g_tx_ring.wr) {
        can_tx_message_type tx;
        mcu_can_msg_t* frame;
        uint8_t mailbox;
        uint8_t i;

        frame = &g_tx_ring.slot[g_tx_ring.rd];
        tx.id_type = (frame->ide != 0u) ? CAN_ID_EXTENDED : CAN_ID_STANDARD;
        tx.frame_type = (frame->rtr != 0u) ? CAN_TFT_REMOTE : CAN_TFT_DATA;
        tx.standard_id = frame->std_id;
        tx.extended_id = frame->ext_id;
        tx.dlc = frame->dlc;
        for (i = 0; i < MCU_CAN_DLEN; ++i) {
            tx.data[i] = frame->data[i];
        }

        mailbox = can_message_transmit(CAN1, &tx);
        if (mailbox == CAN_TX_STATUS_NO_EMPTY) {
            break; /* all mailboxes busy: leave the rest queued for the next pass */
        }
        g_tx_ring.rd = (g_tx_ring.rd + 1u) & MCU_CAN_RING_MASK;
    }
}

void can_send(uint16_t std_id, const uint8_t* data, uint8_t len) {
    uint32_t next;

    if (len > MCU_CAN_DLEN) {
        return;
    }

    next = (g_tx_ring.wr + 1u) & MCU_CAN_RING_MASK;
    if (next != g_tx_ring.rd) { /* drop when the TX ring is full */
        mcu_can_msg_t* frame = &g_tx_ring.slot[g_tx_ring.wr];
        uint8_t i;

        frame->std_id = std_id;
        frame->ext_id = 0;
        frame->ide = 0;
        frame->rtr = 0;
        frame->dlc = len;
        frame->fmi = 0;
        for (i = 0; i < MCU_CAN_DLEN; ++i) {
            frame->data[i] = ((data != NULL) && (i < len)) ? data[i] : 0u;
        }
        g_tx_ring.wr = next;
    }
}

uint8_t can_rx_pop(mcu_can_msg_t* out) {
    uint8_t got = 0;

    if ((out != NULL) && (g_rx_ring.rd != g_rx_ring.wr)) {
        prv_frame_copy(out, &g_rx_ring.slot[g_rx_ring.rd]);
        g_rx_ring.rd = (g_rx_ring.rd + 1u) & MCU_CAN_RING_MASK;
        got = 1;
    }
    return got;
}
