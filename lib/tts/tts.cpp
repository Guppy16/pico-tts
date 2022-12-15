#include "tts.h"
#include "config.h"

// DMA

int tts::dma_chan = dma_claim_unused_channel(true);

dma_channel_config tts::dma_conf = dma_channel_get_default_config(tts::dma_chan);

// PWM

uint tts::pwm_slice_num = pwm_gpio_to_slice_num(MOTOR_GPIO);

uint tts::pwm_channel = pwm_gpio_to_channel(MOTOR_GPIO);

// DShot Command

uint16_t tts::throttle_code = 0;
uint16_t tts::telemetry = 0;

