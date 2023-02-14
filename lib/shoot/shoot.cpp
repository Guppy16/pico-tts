#include "shoot.h"

#include "dshot.h"

uint32_t shoot::dma_buffer[DSHOT_FRAME_LENGTH] = {0};
uint32_t shoot::temp_dma_buffer[DSHOT_FRAME_LENGTH] = {0};

uint16_t shoot::throttle_code = 0;
uint16_t shoot::telemetry = 0;

uint16_t shoot::writes_to_dma_buffer = 0;
uint16_t shoot::writes_to_temp_dma_buffer = 0;

bool shoot::_dma_alarm_rt_state = false;
struct repeating_timer shoot::send_frame_rt;

bool shoot::_uart_telem_rt_state = false;
struct repeating_timer shoot::uart_telem_req_rt;

void shoot::dshot_rt_setup() {
  shoot::_dma_alarm_rt_state = alarm_pool_add_repeating_timer_us(
      tts::pico_alarm_pool, DMA_ALARM_PERIOD, shoot::repeating_send_dshot_frame,
      NULL, &shoot::send_frame_rt);
}

void shoot::send_dshot_frame(bool debug) {
  // Stop timer interrupt JIC
  // irq_set_enabled(DMA_ALARM_IRQ, false);

  // TODO: add more verbose debugging
  // NOTE: caution as this function is executed as an interrupt service
  // routine
  if (debug) {
    printf("Throttle Code: %i", shoot::throttle_code);
  }

  // IF DMA is busy, then write to temp_dma_buffer
  // AND wait for DMA buffer to finish transfer
  // Then copy the temp buffer to dma buffer
  // (NOTE: waiting is risky because this is used in an interrupt)

  if (dma_channel_is_busy(tts::dma_channel)) {
    DShot::command_to_pwm_buffer(shoot::throttle_code, shoot::telemetry,
                                 shoot::temp_dma_buffer, DSHOT_LOW, DSHOT_HIGH,
                                 tts::pwm_channel);
    dma_channel_wait_for_finish_blocking(tts::dma_channel);
    memcpy(shoot::dma_buffer, shoot::temp_dma_buffer,
           DSHOT_FRAME_LENGTH * sizeof(uint32_t));
    ++shoot::writes_to_temp_dma_buffer;
  }
  // ELSE write to dma_buffer directly
  else {
    DShot::command_to_pwm_buffer(shoot::throttle_code, shoot::telemetry,
                                 shoot::dma_buffer, DSHOT_LOW, DSHOT_HIGH,
                                 tts::pwm_channel);
    ++shoot::writes_to_dma_buffer;
  }
  // Re-configure DMA and trigger transfer
  dma_channel_configure(
      tts::dma_channel, &tts::dma_config,
      &pwm_hw->slice[tts::pwm_slice_num].cc, // Write to PWM counter compare
      shoot::dma_buffer, DSHOT_FRAME_LENGTH, true);

  // Reset telemetry to limit the number of requests
  // NOTE: should telem be volatile now?
  shoot::telemetry = 0;
}

bool shoot::repeating_send_dshot_frame(struct repeating_timer *rt) {
  /// NOTE: Can use Use rt->... for debug

  // Send DShot frame
  shoot::send_dshot_frame(false);

  // Return true so that timer repeats
  return true;
}

// Telemetry

uint shoot::_telem_baudrate;

void shoot::uart_telem_irq(void) {
  /// NOTE: uart irq is cleared automatically in hw?

  // Read uart
  while (uart_is_readable(UART_MOTOR_TELEMETRY)) {
    printf("UART: %x\n", uart_getc(UART_MOTOR_TELEMETRY));
  }
}

bool shoot::repeating_uart_telem_req(struct repeating_timer *rt) {
  // Set telemetry
  shoot::telemetry = 1;

  // return true to repeat timer
  return true;
}

void shoot::uart_telemetry_setup() {
  // Initialise and Set baudrate
  shoot::_telem_baudrate =
      uart_init(UART_MOTOR_TELEMETRY, BAUDRATE_MOTOR_TELEMETRY);

  // Set GPIO pin mux for RX
  gpio_set_function(GPIO_MOTOR_TELEMETRY, GPIO_FUNC_UART);

  /// MAYBE: Set pull up
  // gpio_pull_up(GPIO_MOTOR_TELEMETRY);

  // Turn off flow control
  uart_set_hw_flow(UART_MOTOR_TELEMETRY, false, false);

  /// MAYBE: Set data format
  // uart_set_format(UART_MOTOR_TELEMETRY, 8, 1, UART_PARITY_NONE);

  /// TODO: Set IRQ

  // Setup repeating timer for uart telem request
  shoot::_uart_telem_rt_state = alarm_pool_add_repeating_timer_us(
      tts::pico_alarm_pool, UART_TELEMETRY_PERIOD,
      shoot::repeating_uart_telem_req, NULL, &shoot::uart_telem_req_rt);

  /// -- TODO:
  /// MAYBES:
  // check if it has a shared handler
  // check it's priority

  // Add exclusive interrupt handler
  irq_set_exclusive_handler(UART_MOTOR_IRQ, shoot::uart_telem_irq);

  irq_set_enabled(UART_MOTOR_IRQ, true);

  // Enable uart interrupt on rx
  uart_set_irq_enables(UART_MOTOR_TELEMETRY, true, false);
}

void shoot::print_send_frame_rt_setup() {
  printf("\nRepeating Timer Config\n");
  printf("DMA Repeating Timer Setup: %d\n", shoot::_dma_alarm_rt_state);
  printf("Delay: Expected: %li Actual: %li", DMA_ALARM_PERIOD,
         shoot::send_frame_rt.delay_us);
  printf("\tAlarm ID: %i\n", shoot::send_frame_rt.alarm_id);
}

void shoot::print_uart_telem_setup() {
  printf("\nUART Telemetry setup\n");
  printf("Baudrate: %i\n", shoot::_telem_baudrate);
  printf("GPIO Pull Up: %i Down %i\n", gpio_is_pulled_up(GPIO_MOTOR_TELEMETRY),
         gpio_is_pulled_down(GPIO_MOTOR_TELEMETRY));

  // Check uart rx interrupt

  // check if it has a shared handler
  // check it's priority

  // Could be useful to check if uart is enabled?
  printf("UART irq Enabled: %i", irq_is_enabled(UART_MOTOR_IRQ));
}