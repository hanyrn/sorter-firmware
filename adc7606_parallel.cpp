// adc7606_parallel.cpp
//
// Implementation of the AD7606 parallel-bus driver (Cortex-M4).

#include "adc7606_parallel.h"

// ---------------------------------------------------------------------------
// Fast single-store pin helpers (BSRR: low 16 bits set, high 16 bits reset).
// ---------------------------------------------------------------------------
static inline void pinLow(GPIO_TypeDef* port, uint32_t bit)  { port->BSRR = (1u << (bit + 16u)); }
static inline void pinHigh(GPIO_TypeDef* port, uint32_t bit) { port->BSRR = (1u << bit); }
static inline bool pinRead(GPIO_TypeDef* port, uint32_t bit) { return (port->IDR & (1u << bit)) != 0u; }

void Adc7606Parallel::begin() {
    for (uint8_t i = 0; i < AD7606_NUM_DB_LINES; i++) {
        pinMode(AD7606_DB_LINES[i].pin, INPUT);
    }
    pinMode(AD7606_CONVST_PIN, OUTPUT);
    pinMode(AD7606_RESET_PIN,  OUTPUT);
    pinMode(AD7606_RD_PIN,     OUTPUT);
    pinMode(AD7606_CS_PIN,     OUTPUT);
    pinMode(AD7606_BUSY0_PIN,  INPUT);
    pinMode(AD7606_BUSY1_PIN,  INPUT);

    // Idle levels: CS high (both modules released), RD high, CONVST low,
    // RESET inactive.
    pinLow(AD7606_CONVST_PORT, AD7606_CONVST_BIT);
    pinHigh(AD7606_RD_PORT,    AD7606_RD_BIT);
    pinHigh(AD7606_CS_PORT,    AD7606_CS_BIT);
#if AD7606_RESET_ACTIVE_HIGH
    pinLow(AD7606_RESET_PORT, AD7606_RESET_BIT);
#else
    pinHigh(AD7606_RESET_PORT, AD7606_RESET_BIT);
#endif
}

void Adc7606Parallel::reset() {
#if AD7606_RESET_ACTIVE_HIGH
    pinHigh(AD7606_RESET_PORT, AD7606_RESET_BIT);
    tinyDelayNs(100);
    pinLow(AD7606_RESET_PORT, AD7606_RESET_BIT);
#else
    pinLow(AD7606_RESET_PORT, AD7606_RESET_BIT);
    tinyDelayNs(100);
    pinHigh(AD7606_RESET_PORT, AD7606_RESET_BIT);
#endif
    delayMicroseconds(2);
}

// Latch one module's DB0..DB15 from the GPIO input registers. The 16 lines do
// not have to be contiguous: every entry of AD7606_DB_LINES carries its own
// port and bit, so this expands to 16 register bit reads (well under the 100 us
// sample period of 10 kSPS). DB_i of the module ends up in bit i of the word.
static inline uint16_t readWord(const Ad7606DbLine* lines) {
    uint16_t word = 0;
    for (uint8_t i = 0; i < AD7606_DB_PER_MODULE; i++) {
        word |= (uint16_t)((lines[i].port->IDR >> lines[i].bit) & 1u) << i;
    }
    return word;
}

void Adc7606Parallel::convstPulse() {
    pinLow(AD7606_CONVST_PORT, AD7606_CONVST_BIT);
    tinyDelayNs(AD7606_CONVST_PULSE_NS);
    pinHigh(AD7606_CONVST_PORT, AD7606_CONVST_BIT);   // rising edge starts conversion
    tinyDelayNs(AD7606_CONVST_PULSE_NS);
    pinLow(AD7606_CONVST_PORT, AD7606_CONVST_BIT);
}

void Adc7606Parallel::waitReady() {
    const uint32_t t0 = micros();
    while (pinRead(AD7606_BUSY0_PORT, AD7606_BUSY0_BIT) ||
           pinRead(AD7606_BUSY1_PORT, AD7606_BUSY1_BIT)) {
        if ((uint32_t)(micros() - t0) > AD7606_BUSY_TIMEOUT_US) {
            break;   // safety: never block forever if a module is absent
        }
    }
}

void Adc7606Parallel::readAll(int32_t channels[AD7606_NUM_CHANNELS]) {
    convstPulse();           // both modules convert at the same instant
    waitReady();             // wait for both BUSY lines to fall

    // Shared CS + shared RD: every strobe advances BOTH modules to the next
    // channel, and each module presents that channel on its own 16 data lines,
    // so one burst reads channel n of module 0 and module 1 together. Nothing
    // is three-stated and no per-module sequencing is needed.
    pinLow(AD7606_CS_PORT, AD7606_CS_BIT);
    for (uint8_t ch = 0; ch < AD7606_CH_PER_MODULE; ch++) {
        pinLow(AD7606_RD_PORT, AD7606_RD_BIT);       // RD low -> next channel on both buses
        tinyDelayNs(AD7606_TACC_NS);
        channels[ch] = (int16_t)readWord(&AD7606_DB_LINES[0]);
        channels[AD7606_CH_PER_MODULE + ch] =
            (int16_t)readWord(&AD7606_DB_LINES[AD7606_DB_PER_MODULE]);
        pinHigh(AD7606_RD_PORT, AD7606_RD_BIT);
        tinyDelayNs(AD7606_TRD_NS);
    }
    pinHigh(AD7606_CS_PORT, AD7606_CS_BIT);   // release both buses
}
