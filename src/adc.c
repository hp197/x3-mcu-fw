/**
 * \file            adc.c
 * \brief           ADC1 acquisition: phase-current, bus-voltage and throttle sensing
 * \note            Two groups share ADC1. The ordinary group is a 5-channel slow-monitor scan
 *                      (bus voltage, throttle and temperatures) that free-runs under software
 *                      trigger and is copied to a RAM ring by DMA. The preempt group samples the
 *                      three phase currents (ch5/6/7 on PA5/PA6/PA7) synchronously on every TMR1
 *                      CH4 event and raises the preempt-complete interrupt, so the current loop
 *                      reads the three phases at the centre of the PWM period.
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 */
#include "adc.h"

#include "at32f415.h"

/* ------------------------------------------------------------------------- */
/* Channel map                                                               */
/* ------------------------------------------------------------------------- */

/* Ordinary (slow-monitor) scan, in sequence order. PA0, PC5, PA4, PC4, PA3. */
#define ADC_ORD_CH_1    ADC_CHANNEL_0  /*!< rank 1: PA0  */
#define ADC_ORD_CH_2    ADC_CHANNEL_15 /*!< rank 2: PC5  */
#define ADC_ORD_CH_3    ADC_CHANNEL_4  /*!< rank 3: PA4  */
#define ADC_ORD_CH_4    ADC_CHANNEL_14 /*!< rank 4: PC4  */
#define ADC_ORD_CH_5    ADC_CHANNEL_3  /*!< rank 5: PA3  */

/* Preempt (phase-current) group. PA5, PA6, PA7. */
#define ADC_PRE_CH_U    ADC_CHANNEL_5 /*!< preempt 1: PA5 */
#define ADC_PRE_CH_V    ADC_CHANNEL_6 /*!< preempt 2: PA6 */
#define ADC_PRE_CH_W    ADC_CHANNEL_7 /*!< preempt 3: PA7 */

/** \brief DMA ring: ADC_ORDINARY_DEPTH scans of ADC_ORDINARY_RANKS channels, circular. */
#define ADC_DMA_SAMPLES (ADC_ORDINARY_RANKS * ADC_ORDINARY_DEPTH)

/** \brief Ordinary-group DMA ring, filled continuously from the ADC1 ordinary data register. */
static volatile uint16_t prv_dma_ring[ADC_DMA_SAMPLES];

/**
 * \brief           Configure the analog input pins as analog mode
 * \note            PA0/PA3/PA4/PA5/PA6/PA7 on GPIOA, PC4/PC5 on GPIOC.
 */
static void prv_adc_pins_init(void) {
    gpio_init_type gpio_init_struct;

    gpio_default_para_init(&gpio_init_struct);
    gpio_init_struct.gpio_mode = GPIO_MODE_ANALOG;

    gpio_init_struct.gpio_pins = GPIO_PINS_0 | GPIO_PINS_3 | GPIO_PINS_4 | GPIO_PINS_5 | GPIO_PINS_6 | GPIO_PINS_7;
    gpio_init(GPIOA, &gpio_init_struct);

    gpio_init_struct.gpio_pins = GPIO_PINS_4 | GPIO_PINS_5;
    gpio_init(GPIOC, &gpio_init_struct);
}

/**
 * \brief           Wire DMA1 channel 1 to the ADC1 ordinary data register (circular)
 */
static void prv_adc_dma_init(void) {
    dma_init_type dma_init_struct;

    dma_reset(DMA1_CHANNEL1);
    dma_default_para_init(&dma_init_struct);
    dma_init_struct.peripheral_base_addr = (uint32_t)&ADC1->odt;
    dma_init_struct.memory_base_addr = (uint32_t)prv_dma_ring;
    dma_init_struct.direction = DMA_DIR_PERIPHERAL_TO_MEMORY;
    dma_init_struct.buffer_size = ADC_DMA_SAMPLES;
    dma_init_struct.peripheral_inc_enable = FALSE;
    dma_init_struct.memory_inc_enable = TRUE;
    dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_HALFWORD;
    dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_HALFWORD;
    dma_init_struct.loop_mode_enable = TRUE;
    dma_init_struct.priority = DMA_PRIORITY_HIGH;
    dma_init(DMA1_CHANNEL1, &dma_init_struct);

    dma_interrupt_enable(DMA1_CHANNEL1, DMA_FDT_INT, TRUE);
    dma_channel_enable(DMA1_CHANNEL1, TRUE);
}

/**
 * \brief           Configure the ordinary (slow-monitor) 5-channel scan
 */
static void prv_adc_ordinary_init(void) {
    adc_base_config_type adc_base_struct;

    adc_base_default_para_init(&adc_base_struct);
    adc_base_struct.sequence_mode = TRUE; /* scan the 5 ranks in order         */
    adc_base_struct.repeat_mode = TRUE;   /* continuous conversion             */
    adc_base_struct.data_align = ADC_RIGHT_ALIGNMENT;
    adc_base_struct.ordinary_channel_length = ADC_ORDINARY_RANKS;
    adc_base_config(ADC1, &adc_base_struct);

    adc_ordinary_channel_set(ADC1, ADC_ORD_CH_1, 1, ADC_SAMPLETIME_71_5);
    adc_ordinary_channel_set(ADC1, ADC_ORD_CH_2, 2, ADC_SAMPLETIME_71_5);
    adc_ordinary_channel_set(ADC1, ADC_ORD_CH_3, 3, ADC_SAMPLETIME_71_5);
    adc_ordinary_channel_set(ADC1, ADC_ORD_CH_4, 4, ADC_SAMPLETIME_71_5);
    adc_ordinary_channel_set(ADC1, ADC_ORD_CH_5, 5, ADC_SAMPLETIME_71_5);

    adc_dma_mode_enable(ADC1, TRUE);
    adc_ordinary_conversion_trigger_set(ADC1, ADC12_ORDINARY_TRIG_SOFTWARE, TRUE);
}

/**
 * \brief           Configure the preempt (phase-current) group, TMR1 CH4 triggered
 */
static void prv_adc_preempt_init(void) {
    adc_preempt_channel_length_set(ADC1, 3);
    adc_preempt_channel_set(ADC1, ADC_PRE_CH_U, 1, ADC_SAMPLETIME_7_5);
    adc_preempt_channel_set(ADC1, ADC_PRE_CH_V, 2, ADC_SAMPLETIME_7_5);
    adc_preempt_channel_set(ADC1, ADC_PRE_CH_W, 3, ADC_SAMPLETIME_7_5);

    /* The TMR1 CH4 event fires the three-phase sample at the PWM centre. */
    adc_preempt_conversion_trigger_set(ADC1, ADC12_PREEMPT_TRIG_TMR1CH4, TRUE);
    adc_interrupt_enable(ADC1, ADC_PCCE_INT, TRUE);
}

/**
 * \brief           Run the ADC1 power-up calibration
 */
static void prv_adc_calibrate(void) {
    adc_calibration_init(ADC1);
    while (adc_calibration_init_status_get(ADC1) == SET) {}
    adc_calibration_start(ADC1);
    while (adc_calibration_status_get(ADC1) == SET) {}
}

/**
 * \brief           Initialise ADC1 acquisition (ordinary DMA scan + preempt current group)
 */
void adc_init(void) {
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOC_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_ADC1_PERIPH_CLOCK, TRUE);
    crm_adc_clock_div_set(CRM_ADC_DIV_4); /* ADCCLK = PCLK2 / 4, within the converter limit */

    prv_adc_pins_init();
    prv_adc_dma_init();
    prv_adc_ordinary_init();
    prv_adc_preempt_init();

    adc_enable(ADC1, TRUE);
    prv_adc_calibrate();

    adc_ordinary_software_trigger_enable(ADC1, TRUE);
}

/**
 * \brief           Latest raw phase-current sample from the preempt group
 */
uint16_t adc_phase_current_raw(adc_phase_t phase) {
    uint16_t val = 0;

    switch (phase) {
        case ADC_PHASE_U: val = (uint16_t)ADC1->pdt1; break;
        case ADC_PHASE_V: val = (uint16_t)ADC1->pdt2; break;
        case ADC_PHASE_W: val = (uint16_t)ADC1->pdt3; break;
        default: break;
    }
    return val;
}

/**
 * \brief           Mean of the ordinary DMA ring for one scan rank
 */
uint16_t adc_ordinary_mean(uint8_t rank) {
    uint32_t sum = 0;
    uint16_t result = 0;

    if (rank < ADC_ORDINARY_RANKS) {
        uint16_t depth = 0;

        for (depth = 0; depth < ADC_ORDINARY_DEPTH; depth++) {
            sum += prv_dma_ring[(depth * ADC_ORDINARY_RANKS) + rank];
        }
        result = (uint16_t)(sum / ADC_ORDINARY_DEPTH);
    }
    return result;
}
