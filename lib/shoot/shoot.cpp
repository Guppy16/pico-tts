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

void shoot::dshot_rt_setup()
{
    shoot::_dma_alarm_rt_state = alarm_pool_add_repeating_timer_us(tts::pico_alarm_pool, DMA_ALARM_PERIOD, shoot::repeating_send_dshot_frame, NULL, &shoot::send_frame_rt);
}

void shoot::send_dshot_frame(bool debug)
{
    // TODO: add more verbose debugging
    // NOTE: caution as this function is executed as an interrupt service routine
    if (debug)
    {
        Serial.print("Throttle Code: ");
        Serial.println(shoot::throttle_code);
    }

    // IF DMA is busy, then write to temp_dma_buffer
    // AND wait for DMA buffer to finish transfer
    // Then copy the temp buffer to dma buffer
    // (NOTE: waiting is risky because this is used in an interrupt)
    if (dma_channel_is_busy(tts::dma_channel))
    {
        DShot::command_to_pwm_buffer(shoot::throttle_code, shoot::telemetry, shoot::temp_dma_buffer, DSHOT_LOW, DSHOT_HIGH, tts::pwm_channel);
        dma_channel_wait_for_finish_blocking(tts::dma_channel);
        memcpy(shoot::dma_buffer, shoot::temp_dma_buffer, DSHOT_FRAME_LENGTH * sizeof(uint32_t));
        ++shoot::writes_to_temp_dma_buffer;
    }
    // ELSE write to dma_buffer directly
    else
    {
        DShot::command_to_pwm_buffer(shoot::throttle_code, shoot::telemetry, shoot::dma_buffer, DSHOT_LOW, DSHOT_HIGH, tts::pwm_channel);
        ++shoot::writes_to_dma_buffer;
    }
    // Re-configure DMA and trigger transfer
    dma_channel_configure(
        tts::dma_channel,
        &tts::dma_config,
        &pwm_hw->slice[tts::pwm_slice_num].cc, // Write to PWM counter compare
        shoot::dma_buffer,
        DSHOT_FRAME_LENGTH,
        true);

    // Reset telemetry to limit the number of requests
    // NOTE: should telem be volatile now?
    shoot::telemetry = 0;
}

bool shoot::repeating_send_dshot_frame(struct repeating_timer *rt)
{
    /// NOTE: Can use Use rt->... for debug

    // Send DShot frame
    shoot::send_dshot_frame(false);

    // Return true so that timer repeats
    return true;
}


// Telemetry

uint shoot::_telem_baudrate;

void uart_telem_irq(void)
{
    // Clear uart irq?

    // Read uart

    // 
}

bool shoot::repeating_uart_telem_req(struct repeating_timer *rt)
{
    // Set telemetry
    shoot::telemetry = 1;

    // return true to repeat timer
    return true;
}

void shoot::uart_telemetry_setup()
{
    // Initialise and Set baudrate
    shoot::_telem_baudrate = uart_init(UART_MOTOR_TELEMETRY, BAUDRATE_MOTOR_TELEMETRY);

    // Set GPIO pin mux for RX
    gpio_set_function(GPIO_MOTOR_TELEMETRY, GPIO_FUNC_UART);
    
    // Set pull up
    // gpio_pull_up(GPIO_MOTOR_TELEMETRY);

    // Setup repeating timer for uart telem request
    shoot::_uart_telem_rt_state = alarm_pool_add_repeating_timer_us(tts::pico_alarm_pool, UART_TELEMETRY_PERIOD, shoot::repeating_uart_telem_req, NULL, &shoot::uart_telem_req_rt);

    /// -- TODO:
    /// MAYBES:
    // check if it has a shared handler
    // check it's priority

    // Add exclusive interrupt handler to uart0 (UART_MOTOR_TELEMETRY), UART0_IRQ

    // Enable uart interrupt (uart_set_irq_enables)
    
}

void shoot::print_shoot_setup()
{
    Serial.println("\nRepeating Timer Config");
    Serial.print("Repeating Timer Setup: ");
    Serial.print(shoot::_dma_alarm_rt_state);
    Serial.print("Alarm ID: ");
    Serial.print(shoot::send_frame_rt.alarm_id);
    Serial.print("Alarm Period (us): ");
    Serial.print("Expected: ");
    Serial.println(DMA_ALARM_PERIOD);
    Serial.print("\tActual: ");
    Serial.println(shoot::send_frame_rt.delay_us);

    Serial.print("Initial Throttle Code: ");
    Serial.print(shoot::throttle_code);
    Serial.print("\tTelemetry: ");
    Serial.println(shoot::telemetry);
}


void shoot::print_uart_telem_setup()
{
    Serial.println("\nUART telemetry setup");

    Serial.print("Baudrate: ");
    Serial.println(shoot::_telem_baudrate);

    // Check if gpio is pulled up
    Serial.print("Gpio pull: ");
    Serial.print("Up: ");
    Serial.print(gpio_is_pulled_up(GPIO_MOTOR_TELEMETRY));

    Serial.print("\tDown: ");
    Serial.print(gpio_is_pulled_down(GPIO_MOTOR_TELEMETRY));
}
