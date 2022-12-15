#include "logging.h"
#include "config.h"
#include "tts.h"

void logging::dshot_setup()
{
    Serial.println("\nDShot Setup");

    Serial.print("Wrap: ");
    Serial.print(DMA_WRAP);

    Serial.print("\tLow: ");
    Serial.print(DSHOT_LOW);

    Serial.print("\tHigh: ");
    Serial.print(DSHOT_HIGH);

    Serial.println();
}

void logging::gpio_setup()
{
    Serial.println("\nGPIO Setup");

    Serial.print("LED_BUILTIN: GPIO ");
    Serial.println(LED_BUILTIN);

    Serial.print("MOTOR: GPIO ");
    Serial.println(MOTOR_GPIO);
}

void logging::pwm_setup()
{
    Serial.println("\nPWM Setup");

    Serial.print("Slice Num: ");
    Serial.println(tts::pwm_slice_num);

    Serial.print("Channel: ");
    Serial.println(tts::pwm_channel);
}

void logging::dma_setup()
{
    Serial.println("\nDMA Setup");

    Serial.print("Channel: ");
    Serial.println(tts::dma_chan);

    Serial.print("Buffer Length: ");
    Serial.println(DSHOT_FRAME_LENGTH);

    Serial.print("Repeating Timer Setup: ");
    Serial.println("Not implemented");
    // Serial.print(tts::dma_alarm_rt_state);

    Serial.print("Alarm Period (us): ");
    Serial.println(DMA_ALARM_PERIOD);
}

void logging::flash_led(const pin_size_t &pin)
{
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);
}

// --- Deprecated functions

void logging::print_pwm_config(const pwm_config &conf)
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