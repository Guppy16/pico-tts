#pragma once
#include <Arduino.h>

namespace logging
{
    void all_setup();

    /*! \brief DShot settings: wrap, low, high
     *  Ambiguous whether this should go in pwm_setup or dshot setup
     */
    void dshot_setup();

    /*! \brief Print GPIO settings for builtin led, motor,
     */
    void gpio_setup();

    /*! \brief Print PWM slice, channel
     */
    void pwm_setup();

    /*! \brief Print dma channel, buffer length
     */
    void dma_setup();

    /*! \brief Flash LEDs
     *
     *  Useful to check the responsiveness of the system,
     *  albeit this is a blocking-process :|
     * 
     *  TODO: another non-blocking version of this function can be 
     *  implemented using timer interrupts
     *
     *  \param pin is the GPIO pin of the LED. Default is LED_BUILTIN
     */
    void flash_led(const pin_size_t &pin = LED_BUILTIN);

    // --- Depracated functions

    /*! \brief Print PWM config: csr, div, top
     */
    void print_pwm_config(const pwm_config &conf);

}