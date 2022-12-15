#pragma once
#include "pico/time.h"
#include "pico/time.h"
#include "hardware/dma.h"
#include "hardware/pwm.h"

namespace tts
{
    // DMA config
    extern int dma_chan;
    extern dma_channel_config dma_conf;

    // PWM config

    /*! \brief The slice number is the upper or lower half of 32 bits
     *  \ingroup PWM
     *
     * Timers are 16 bits, but are inefficint to store these in 32 bit systems
     * hence timers are stored in either the lower or upper "slice" of 32 bits
     */
    extern uint pwm_slice_num;
    extern uint pwm_channel;

    // DShot Command
    extern uint16_t throttle_code;
    extern uint16_t telemetry;

    // FUNCTIONS

    /*! \brief Routine to send a DShot Frame over DMA
     *
     *  \param debug set true for serial logs of each frame sent
     */
    void send_dshot_frame(bool debug = true);

    /*! \brief ISR to send DShot frame over DMA channel
     *
     *  \param rt reapeating timer configuration
     *
     *  \return True if the timer wants to repeat
     */
    bool repeating_send_dshot_frame(struct repeating_timer *rt);

}