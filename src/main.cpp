#include <Arduino.h>

#include "shoot.h"
#include "utils.h"

void setup()
{
    Serial.begin(115200);

    // Flash LED
    utils::flash_led(LED_BUILTIN);

    // pwm config
    // Note that PWM needs to be setup first,
    // because the dma dreq requires tts::pwm_slice_num
    tts::pwm_setup();

    // dma config
    tts::dma_setup();

    // Set repeating timer
    // NOTE: this can be put in main loop to start
    // repeating timer on key press (e.g. a for arm)
    shoot::rt_setup();

    // Setup UART
    tts::uart_telemetry_setup();

    delay(1500);

    tts::print_gpio_setup();
    tts::print_dshot_setup();
    tts::print_pwm_setup();
    tts::print_dma_setup();
    tts::print_uart_telem_setup();
    shoot::print_shoot_setup();
}

int incomingByte;
char incomingChar;

void loop()
{
    // Read from uart:
    if (uart_is_readable(UART_MOTOR_TELEMETRY))
    {
        // print uart to serial
        Serial.print("UART: ");
        incomingChar = uart_getc(UART_MOTOR_TELEMETRY);
        Serial.println(incomingChar, BIN);
    }

    if (Serial.available() > 0)
    {
        incomingByte = Serial.read();
        Serial.print("Byte: ");
        Serial.println(incomingByte, DEC);

        // l (led)
        if (incomingByte == 108)
        {
            utils::flash_led(LED_BUILTIN);
        }

        // a (arm)
        // if (incomingByte == 97)
        // {
        //     // Disable timer
        //     cancel_repeating_timer(&send_frame_rt);
        //     // Arm motor
        //     arm_motor();
        //     // Re-enable timer
        //     _dma_alarm_rt_state = alarm_pool_add_repeating_timer_us(pico_alarm_pool, DMA_ALARM_PERIOD, repeating_send_dshot_frame, NULL, &send_frame_rt);
        //     Serial.print("Re-enabled repeating DMA alarm: ");
        //     Serial.println(_dma_alarm_rt_state);

        //     return;
        // }

        // b - beep
        if (incomingByte == 98)
        {
            shoot::throttle_code = 1;
            shoot::telemetry = 1;
        }

        // o - on
        if (incomingByte == 111)
        {
            shoot::throttle_code = 22;
            shoot::telemetry = 1;
        }

        // t - telemetry
        if (incomingByte == 116)
        {
            // shoot::telemetry = 1 - shoot::telemetry;
            // Serial.print("Telemetry: ");
            // Serial.println(shoot::telemetry);
            shoot::telemetry = 1;
            shoot::send_dshot_frame();
            if (uart_is_readable(UART_MOTOR_TELEMETRY))
            {
                // print uart to serial
                Serial.print("UART: ");
                incomingChar = uart_getc(UART_MOTOR_TELEMETRY);
                Serial.println(incomingChar, BIN);
            }
            shoot::telemetry = 0;
        }

        // s - spin
        // if (incomingByte == 115)
        // {
        //     // Disable timer
        //     cancel_repeating_timer(&send_frame_rt);
        //     // Ramp up to speed ARM_THROTTLE + 100
        //     ramp_motor();
        //     // Re-enable timer
        //     _dma_alarm_rt_state = alarm_pool_add_repeating_timer_us(pico_alarm_pool, DMA_ALARM_PERIOD, repeating_send_dshot_frame, NULL, &send_frame_rt);
        //     Serial.print("Re-enabled repeating DMA alarm: ");
        //     Serial.println(_dma_alarm_rt_state);
        // }

        // r - rise
        if (incomingByte == 114)
        {
            if (shoot::throttle_code >= ZERO_THROTTLE and shoot::throttle_code <= MAX_THROTTLE)
            {
                // Check for max throttle
                if (shoot::throttle_code == MAX_THROTTLE)
                {
                    Serial.println("Max Throttle reached");
                }
                else
                {
                    shoot::throttle_code += 1;
                    Serial.print("Throttle: ");
                    Serial.println(shoot::throttle_code);
                }
            }
            else
            {
                Serial.println("Motor is not in throttle mode");
            }
        }

        // f - fall
        if (incomingByte == 102)
        {
            if (shoot::throttle_code <= MAX_THROTTLE && shoot::throttle_code >= ZERO_THROTTLE)
            {
                if (shoot::throttle_code == ZERO_THROTTLE)
                {
                    Serial.println("Throttle is zero");
                }
                else
                {
                    shoot::throttle_code -= 1;
                    Serial.print("Throttle: ");
                    Serial.println(shoot::throttle_code);
                }
            }
            else
            {
                Serial.println("Motor is not in throttle mode");
            }
        }

        // spacebar = 0 throttle --> This could be disarm.
        // Not sure how to disarm yet. Maybe set throttle to 0 and don't send a cmd for some secs?
        if (incomingByte == 32)
        {
            shoot::throttle_code = ZERO_THROTTLE;
            shoot::telemetry = 0;
            Serial.println("Throttle: 0");
        }

        // NOTE: In DEBUG mode, sending a DSHOT Frame takes a lot of time!
        // So it may seem as if the PICO is unable to detect key presses
        // while sending commands!
        // But is this even needed?
        shoot::send_dshot_frame();

        Serial.println("Finished processing byte.");
    }
}