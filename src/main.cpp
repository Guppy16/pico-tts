#include <Arduino.h>

#include "config.h"
#include "tts.h"
#include "logging.h"

#include "pico/time.h"

#include "hardware/pwm.h"
#include "hardware/dma.h"
#include "hardware/timer.h"
#include "hardware/irq.h"
#include "dshot.h"
#include "utils.h"

// TODO: Does this ever need to be volatile?
uint32_t dma_buffer[DSHOT_FRAME_LENGTH] = {0};

// TODO: Change default behviour to: bool debug = false
void send_dshot_frame(bool = true);

// DMA config
// int dma_chan = dma_claim_unused_channel(true);
// dma_channel_config dma_conf = dma_channel_get_default_config(dma_chan);

int dma_chan = tts::dma_chan;
// dma_channel_config dma_conf = tts::dma_conf;
dma_channel_config dma_conf = dma_channel_get_default_config(tts::dma_chan);


// PWM config
// uint pwm_slice_num = pwm_gpio_to_slice_num(MOTOR_GPIO);
// uint pwm_channel = pwm_gpio_to_channel(MOTOR_GPIO);

uint pwm_slice_num = tts::pwm_slice_num;
uint pwm_channel = tts::pwm_channel;

// DMA Alarm config
// ISR to send DShot frame over DMA
bool repeating_send_dshot_frame(struct repeating_timer *rt)
{
    // Send DShot frame
    send_dshot_frame(false);
    // CAN DO: Use rt-> for debug
    // Return true so that timer repeats
    return true;
}

// Keep track of repeating timer status
bool dma_alarm_rt_state = false;
// Setup a repeating timer configuration
struct repeating_timer send_frame_rt;
// Create alarm pool
alarm_pool_t *pico_alarm_pool = alarm_pool_create(DMA_ALARM_NUM, PICO_TIME_DEFAULT_ALARM_POOL_MAX_TIMERS);

uint16_t throttle_code = 0;
uint16_t telemtry = 0;

void setup()
{
    Serial.begin(9600);

    // Flash LED
    logging::flash_led(LED_BUILTIN);

    // --- Setup PWM config
    // Initialise PWM to ouput 0 signal
    // TODO: start PWM after DMA config
    gpio_set_function(MOTOR_GPIO, GPIO_FUNC_PWM);
    pwm_set_wrap(pwm_slice_num, DSHOT_PWM_WRAP);
    pwm_set_chan_level(pwm_slice_num, pwm_channel, 0);    // Set PWM to 0 output
    pwm_set_clkdiv(pwm_slice_num, DEBUG ? 240.0f : 1.0f); // Should run at 500 kHz for cpu-clck = 120 Mhz
    pwm_set_enabled(pwm_slice_num, true);

    // --- Setup DMA
    // PWM counter compare is 32 bit (16 bit per channel)
    channel_config_set_transfer_data_size(&dma_conf, DMA_SIZE_32);
    // Increment read address
    channel_config_set_read_increment(&dma_conf, true);
    // NO increment write address
    channel_config_set_write_increment(&dma_conf, false);
    // DMA Data request when PWM is finished
    channel_config_set_dreq(&dma_conf, DREQ_PWM_WRAP0 + pwm_slice_num);

    // Set repeating timer
    dma_alarm_rt_state = alarm_pool_add_repeating_timer_us(pico_alarm_pool, DMA_ALARM_PERIOD, repeating_send_dshot_frame, NULL, &send_frame_rt);

    delay(1500);

    // Print General Settings
    logging::all_setup();

    Serial.print("DMA Repeating Timer Setup: ");
    Serial.println(dma_alarm_rt_state);

    Serial.print("Initial throttle: ");
    Serial.print(throttle_code);
    Serial.print("\tInitial telemetry: ");
    Serial.println(telemtry);
}

int incomingByte;
uint32_t temp_dma_buffer[DSHOT_FRAME_LENGTH] = {0};


// Helper function to send code to DMA buffer
volatile uint16_t writes_to_temp_dma_buffer = 0;
volatile uint16_t writes_to_dma_buffer = 0;
void send_dshot_frame(bool debug)
{
    // Stop timer interrupt JIC
    // irq_set_enabled(DMA_ALARM_IRQ, false);

    if (debug)
    {
        Serial.print(throttle_code);
        Serial.print("\t");
    }

    // IF DMA is busy, then write to temp_dma_buffer
    // AND wait for DMA buffer to finish transfer
    // Then copy the temp buffer to dma buffer
    if (dma_channel_is_busy(dma_chan))
    {
        DShot::command_to_pwm_buffer(throttle_code, telemtry, temp_dma_buffer, DSHOT_LOW, DSHOT_HIGH, pwm_channel);
        dma_channel_wait_for_finish_blocking(dma_chan);
        memcpy(dma_buffer, temp_dma_buffer, DSHOT_FRAME_LENGTH * sizeof(uint32_t));
        ++writes_to_temp_dma_buffer;
    }
    // ELSE write to dma_buffer directly
    else
    {
        DShot::command_to_pwm_buffer(throttle_code, telemtry, dma_buffer, DSHOT_LOW, DSHOT_HIGH, pwm_channel);
        ++writes_to_dma_buffer;
    }
    // Re-configure DMA and trigger transfer
    dma_channel_configure(
        dma_chan,
        &dma_conf,
        &pwm_hw->slice[pwm_slice_num].cc, // Write to PWM counter compare
        dma_buffer,
        DSHOT_FRAME_LENGTH,
        true);

    // Restart interrupt
    // irq_set_enabled(DMA_ALARM_IRQ, true);
    // NOTE: Alarm is only 32 bits
    // so, be careful if delay is more than that
    // uint64_t target = timer_hw->timerawl + DMA_ALARM_PERIOD;
    // timer_hw->alarm[DMA_ALARM_NUM] = (uint32_t)target;
}



void ramp_motor()
{
    // Debugging
    writes_to_temp_dma_buffer = 0;
    writes_to_dma_buffer = 0;

    uint64_t duration;    // milli seconds
    uint64_t target_time; // micro seconds
    uint n = 101 - 1;     // Num of commands to send on rise and fall

    // Increase throttle from 0 from to ARM_THROTTLE + 100 in 100 steps
    // < 20 ms time for Dshot 150, 20 bit frame length, n = 100
    uint16_t ramp_throttle = ARM_THROTTLE + 100;
    telemtry = 0;
    for (uint16_t i = 0; i <= n; ++i)
    {
        throttle_code = THROTTLE_ZERO + i * (ramp_throttle - THROTTLE_ZERO) / n;
        send_dshot_frame();
    }

    // Maintain this throttle for 500 ms
    throttle_code = ramp_throttle;
    telemtry = 0;
    duration = 500; // ms
    target_time = timer_hw->timerawl + duration * 1000;
    // TODO: re implement as a timer interrupt
    while (timer_hw->timerawl < target_time)
        send_dshot_frame();
    
    Serial.print("Write to temp dma buffer: ");
    Serial.println(writes_to_temp_dma_buffer);

    Serial.print("Write to dma buffer: ");
    Serial.println(writes_to_dma_buffer);
}

void loop()
{

    if (Serial.available() > 0)
    {
        incomingByte = Serial.read();
        Serial.println(incomingByte, DEC);

        // l (led)
        if (incomingByte == 108)
        {
            logging::flash_led(LED_BUILTIN);
        }

        // a (arm)
        if (incomingByte == 97)
        {
            // Disable timer
            cancel_repeating_timer(&send_frame_rt);
            // Arm motor
            utils::arm_motor();
            // Re-enable timer
            dma_alarm_rt_state = alarm_pool_add_repeating_timer_us(pico_alarm_pool, DMA_ALARM_PERIOD, repeating_send_dshot_frame, NULL, &send_frame_rt);
            Serial.print("Re-enabled repeating DMA alarm: ");
            Serial.println(dma_alarm_rt_state);

            return;
        }

        // b - beep
        if (incomingByte == 98)
        {
            throttle_code = 1;
            telemtry = 1;
        }

        // s - spin
        if (incomingByte == 115)
        {
            // Disable timer
            cancel_repeating_timer(&send_frame_rt);
            // Ramp up to speed ARM_THROTTLE + 100
            ramp_motor();
            // Re-enable timer
            dma_alarm_rt_state = alarm_pool_add_repeating_timer_us(pico_alarm_pool, DMA_ALARM_PERIOD, repeating_send_dshot_frame, NULL, &send_frame_rt);
            Serial.print("Re-enabled repeating DMA alarm: ");
            Serial.println(dma_alarm_rt_state);
        }

        // r - rise
        if (incomingByte == 114)
        {
            if (throttle_code >= THROTTLE_ZERO and throttle_code <= MAX_THROTTLE)
            {
                // Check for max throttle
                if (throttle_code == MAX_THROTTLE)
                {
                    Serial.println("Max Throttle reached");
                }
                else
                {
                    throttle_code += 1;
                    Serial.print("Throttle: ");
                    Serial.println(throttle_code);
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
            if (throttle_code <= MAX_THROTTLE && throttle_code >= THROTTLE_ZERO)
            {
                if (throttle_code == THROTTLE_ZERO)
                {
                    Serial.println("Throttle is zero");
                }
                else
                {
                    throttle_code -= 1;
                    Serial.print("Throttle: ");
                    Serial.println(throttle_code);
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
            throttle_code = THROTTLE_ZERO;
            telemtry = 0;
        }

        send_dshot_frame();
        Serial.println("Finished processing byte.");
    }
}