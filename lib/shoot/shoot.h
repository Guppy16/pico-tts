/*
 *  This file builds off of tts
 *  to send dshot frames
 *  with a repeating timer
 */

#pragma once
#include <stdio.h>
#include <string.h>

#include "hardware/irq.h"
#include "hardware/uart.h"
#include "tts.h"

#define GPIO_MOTOR_TELEMETRY 13
#define UART_MOTOR_TELEMETRY uart0 // Note there are only two uarts available
#define BAUDRATE_MOTOR_TELEMETRY 115200
#define UART_TELEMETRY_PERIOD 1000000 // microseconds

namespace shoot {
// TODO: Does this ever need to be volatile?
extern uint32_t dma_buffer[DSHOT_FRAME_LENGTH];
extern uint32_t temp_dma_buffer[DSHOT_FRAME_LENGTH];

// Declare throttle and telemetry variables
// These are made global for now so that it is easy to modify
extern uint16_t throttle_code;
extern uint16_t telemetry;

extern uint16_t writes_to_temp_dma_buffer;
extern uint16_t writes_to_dma_buffer;

/**
 *  \brief sends a dshot frame with optional debug info.
 *  converts the throttle_code and telemetry to a dshot frame
 *  writes to the frame buffer
 *  re-configures dma
 *
 *  \note A (premature?) optimisation is done by checking
 *  if the dma channel is busy, then calculate the frame
 *  and write to temp dma buffer.
 *  This is transferred to the normal dma_buffer afterwards
 *
 *  \param debug Used to print throttle code to serial
 */
void send_dshot_frame(bool debug = false);

// --- Repeating timer setup for dshot frame

// Keep track of repeating timer status
extern bool _dma_alarm_rt_state;

// Setup a repeating timer configuration
extern struct repeating_timer send_frame_rt;

// Routine to setup repeating timer to send dshot frame
void dshot_rt_setup();

// rt callback to send DShot frame over DMA
bool repeating_send_dshot_frame(struct repeating_timer *rt);

// --- Uart Telem setup

/*! \brief setup single wire telemetry uart
 */
void uart_telemetry_setup();
extern uint _telem_baudrate;
extern bool _uart_telem_rt_state;
// Repeating timer configuration for Uart telemetry request
extern struct repeating_timer uart_telem_req_rt;

/// TODO: uart telem receieve interrupt handler
void uart_telem_irq(void);

// rt callback for uart telemetry request
bool repeating_uart_telem_req(struct repeating_timer *rt);

// Debug functions
void print_send_frame_rt_setup();
void print_uart_telem_setup();

} // namespace shoot
