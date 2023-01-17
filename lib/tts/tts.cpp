#include "tts.h"

// PWM

void tts::pwm_setup()
{
    gpio_set_function(MOTOR_GPIO, GPIO_FUNC_PWM);

    // Wrap is the number of counts in each PWM period
    pwm_set_wrap(tts::pwm_slice_num, DSHOT_PWM_WRAP);

    // 0 duty cycle by default
    pwm_set_chan_level(tts::pwm_slice_num, tts::pwm_channel, 0);

    // Set clock divier
    pwm_set_clkdiv(tts::pwm_slice_num, DSHOT_PWM_DIV);

    // Turn on PWM
    pwm_set_enabled(tts::pwm_slice_num, true);
}

// DMA

dma_channel_config tts::dma_config = dma_channel_get_default_config(tts::dma_channel);

void tts::dma_setup()
{
    // PWM counter compare is 32 bit (16 bit for each pwm channel)
    // NOTE: Technically we are only using one channel,
    // so this will make the other channel useless
    channel_config_set_transfer_data_size(&tts::dma_config, DMA_SIZE_32);
    // Increment read address
    channel_config_set_read_increment(&tts::dma_config, true);
    // NO increment write address
    channel_config_set_write_increment(&tts::dma_config, false);
    // DMA Data request when PWM is finished
    // This is because 16 DMA transfers are required to send a dshot frame
    channel_config_set_dreq(&tts::dma_config, DREQ_PWM_WRAP0 + tts::pwm_slice_num);
}

// Telemetry

uint tts::_telem_baudrate;

void tts::uart_telemetry_setup()
{
    // Initialise and Set baudrate
    tts::_telem_baudrate = uart_init(UART_MOTOR_TELEMETRY, 115200);

    // Set GPIO pin mux for RX
    gpio_set_function(GPIO_MOTOR_TELEMETRY, GPIO_FUNC_UART);
    
    // Set pull up
    // gpio_pull_up(GPIO_MOTOR_TELEMETRY);
}

// Alarm pool

alarm_pool_t *tts::pico_alarm_pool = alarm_pool_create(DMA_ALARM_NUM, PICO_TIME_DEFAULT_ALARM_POOL_MAX_TIMERS);

// Debugging

void tts::print_gpio_setup()
{
    Serial.println("\nGPIO Setup");

    Serial.print("LED_BUILTIN: GPIO ");
    Serial.println(LED_BUILTIN);

    Serial.print("MOTOR: GPIO ");
    Serial.println(MOTOR_GPIO);
}

void tts::print_dshot_setup()
{
    Serial.println("\nDShot Setup");

    Serial.print("Wrap: ");
    Serial.print(DSHOT_PWM_WRAP);

    Serial.print("\tLow: ");
    Serial.print(DSHOT_LOW);

    Serial.print("\tHigh: ");
    Serial.print(DSHOT_HIGH);

    Serial.println();
}

void tts::print_alarm_pool_setup()
{
    Serial.println("\nAlarm Pool Setup");
    Serial.println("Not implmeneted yet");

    // For some reason, this doesn't work?
    // Serial.println((tts::pico_alarm_pool)->hardware_alarm_num);
}

void tts::print_pwm_setup()
{
    Serial.println("\nPWM Setup");

    Serial.print("Slice Num: ");
    Serial.println(tts::pwm_slice_num);

    Serial.print("Channel: ");
    Serial.println(tts::pwm_channel);
}

void tts::print_dma_setup()
{
    Serial.println("\nDMA Setup");

    Serial.print("Channel: ");
    Serial.println(tts::dma_channel);

    Serial.print("Buffer Length: ");
    Serial.println(DSHOT_FRAME_LENGTH);
}

void tts::print_uart_telem_setup()
{
    Serial.println("\nUART telemetry setup");

    Serial.print("Baudrate: ");
    Serial.println(tts::_telem_baudrate);

    // Check if gpio is pulled up
    Serial.print("Gpio pull: ");
    Serial.print("Up: ");
    Serial.print(gpio_is_pulled_up(GPIO_MOTOR_TELEMETRY));

    Serial.print("\tDown: ");
    Serial.print(gpio_is_pulled_down(GPIO_MOTOR_TELEMETRY));
}

// Depracated

void tts::_print_pwm_setup(const pwm_config &conf)
{
    Serial.println("\nPWM config");

    Serial.print("csr: ");
    Serial.print(conf.csr);

    Serial.print("\tdiv: ");
    Serial.print(conf.div);

    Serial.print("\ttop: ");
    Serial.print(conf.top);

    Serial.println();
}