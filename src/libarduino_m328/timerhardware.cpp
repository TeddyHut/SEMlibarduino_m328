/*
 * timerhardware.cpp
 *
 * Created: 24/01/2019 4:10:00 AM
 *  Author: teddy
 */

#include <avr/io.h>
#include <avr/interrupt.h>

#include "../timerhardware.h"

//Note: This will break PWM on PD3 and PB3 (3 and 11)
//Note: This will not compensate for an imperfect division/prescale match. If the cpu_freq and available prescales do not factor to tick_freq, the clock will drift.

//Prescale bits for timer 2
constexpr uint16_t prescaleMap[][2] = {
    {0b001, 1},
    {0b010, 8},
    {0b011, 32},
    {0b100, 64},
    {0b101, 128},
    {0b110, 256},
    {0b111, 1024}
};

struct TimerProperties {
    uint8_t ocr;
    uint8_t prescale_bits;
};

constexpr TimerProperties get_timer_properties(float const cpu_freq, float const tick_freq)
{
    //Determine prescale bits to use
    auto required_presacle = cpu_freq / tick_freq;
    for(uint8_t i = 0; i < sizeof(prescaleMap) / (2 * sizeof(uint16_t)); i++) {
        auto variable_prescale = static_cast<uint32_t>(required_presacle / prescaleMap[i][1]);
        if(variable_prescale <= 0xff) {
            return {static_cast<uint8_t>(variable_prescale), static_cast<uint8_t>(prescaleMap[i][0])};
        }
    }
    return {0, 0};
}

constexpr TimerProperties timer_properties = get_timer_properties(F_CPU, 1000);
static_assert(timer_properties.ocr != 0, "no appropriate TIMER2 prescale found");

ISR(TIMER2_COMPA_vect)
{
    libmodule::time::isr_timer();
}

void libmodule::time::isr_timer()
{
    TimerBase<1000>::handle_isr();
}

void libmodule::time::TimerBase<1000>::handle_isr()
{
    for(il_count_t i = 0; i < il_instances.size(); i++) {
        static_cast<TimerBase *>(il_instances[i])->tick();
    }
}

void libmodule::time::TimerBase<1000>::start_daemon()
{
    //Set timer to CTC mode, enable compare interrupt, set compare value, and start with prescale
    TCCR2A = 1 << WGM21;
    TIMSK2 = 1 << OCIE2A;
    OCR2A = timer_properties.ocr;
    TCCR2B = timer_properties.prescale_bits;
}
