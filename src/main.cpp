#include <stdio.h>
#include <string.h>

#include "hardware/uart.h"
#include "hx711.h"
#include "hx711_noblock.pio.h"
#include "pico/platform.h"
#include "pico/stdlib.h"
#include "shoot.h"
#include "utils.h"

void read_telem_uart() {
  while (uart_is_readable(UART_MOTOR_TELEMETRY)) {
    printf("UART: %x\n", uart_getc(UART_MOTOR_TELEMETRY));
  }
}

void update_signal(const int &key_input) {
  // l (led)
  if (key_input == 108) {
    utils::flash_led(LED_BUILTIN);
  }

  // b - beep
  if (key_input == 98) {
    shoot::throttle_code = 1;
    shoot::telemetry = 1;
  }

  // r - rise
  if (key_input == 114) {
    if (shoot::throttle_code >= ZERO_THROTTLE and
        shoot::throttle_code <= MAX_THROTTLE) {
      shoot::throttle_code =
          MIN(shoot::throttle_code + THROTTLE_INCREMENT, MAX_THROTTLE);

      printf("Throttle: %i\n", shoot::throttle_code - ZERO_THROTTLE);
      shoot::throttle_code == MAX_THROTTLE &&printf("Max Throttle reached\n");

    } else {
      printf("Motor is not in throttle mode\n");
    }
  }

  // f - fall
  if (key_input == 102) {
    if (shoot::throttle_code <= MAX_THROTTLE &&
        shoot::throttle_code >= ZERO_THROTTLE) {
      shoot::throttle_code =
          MAX(shoot::throttle_code - THROTTLE_INCREMENT, ZERO_THROTTLE);

      printf("Throttle: %i\n", shoot::throttle_code - ZERO_THROTTLE);
      shoot::throttle_code == ZERO_THROTTLE &&printf("Throttle is zero\n");

    } else {
      printf("Motor is not in throttle mode\n");
    }
  }

  // spacebar - send zero throttle
  if (key_input == 32) {
    shoot::throttle_code = ZERO_THROTTLE;
    shoot::telemetry = 0;
    printf("Throttle: %i\n", 0);
  }

  // t - telemetry
  if (key_input == 116) {
    shoot::telemetry = 1;
    shoot::send_dshot_frame();
    shoot::telemetry = 0;
    read_telem_uart();
  }

  shoot::send_dshot_frame();
  printf("Finished processing key input %i\n", key_input);
}

void print_load_cell_setup() {
  printf("\nHX711 Load Cell Setup\n");
  printf("Pins: CLK %i DAT %i\n", HX711_CLKPIN, HX711_DATPIN);
}

int main() {
  // Set MCU clock frequency. Should we assert this?
  set_sys_clock_khz(MCU_FREQ * 1e3, false);

  int32_t thrust;
  int key_input;

  stdio_init_all();
  gpio_init(LED_BUILTIN);
  gpio_set_dir(LED_BUILTIN, GPIO_OUT);

  // Flash LED on and off 3 times
  utils::flash_led(LED_BUILTIN, 3);

  // Setup Load cell
  int32_t zero_val = 150622; // Load cell calibration
  hx711_t hx;
  hx711_init(&hx, HX711_CLKPIN, HX711_DATPIN, pio0, &hx711_noblock_program,
             &hx711_noblock_program_init);
  hx711_power_up(&hx, hx711_gain_128);
  hx711_wait_settle(hx711_rate_10);

  // Setup DShot

  // Set repeating timer
  // NOTE: this can be put in main loop to start
  // repeating timer on key press (e.g. a for arm)
  shoot::dshot_rt_setup();

  // Setup UART
  shoot::uart_telemetry_setup();
  // Setup UART
  // tts::uart_telemetry_setup();

  // Note that PWM needs to be setup first,
  // because the dma dreq requires tts::pwm_slice_num
  tts::pwm_setup();
  tts::dma_setup();

  sleep_ms(1500);

  printf("MCU Freq (kHz): Expected %d Actual %d\n", MCU_FREQ * 1000,
         frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS));

  tts::print_gpio_setup();
  tts::print_dshot_setup();
  tts::print_pwm_setup();
  tts::print_dma_setup();
  // tts::print_uart_telem_setup();
  shoot::print_uart_telem_setup();

  shoot::print_send_frame_rt_setup();
  print_load_cell_setup();

  printf("Initial throttle: %i\n", shoot::throttle_code);
  printf("Initial telemetry: %i\n", shoot::telemetry);

  while (1) {
    key_input = getchar_timeout_us(0);
    if (key_input != PICO_ERROR_TIMEOUT) {
      update_signal(key_input);
    }
    sleep_ms(100);
    if (hx711_get_value_noblock(&hx, &thrust) && thrust) {
      printf("hx711: %li\n", thrust - zero_val);
    }
    read_telem_uart();
  }
}
