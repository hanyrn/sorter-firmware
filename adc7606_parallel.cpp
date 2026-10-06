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
    for (uint8_t i = 0; i < AD7606_NUM_CHANNELS; i++) {
        pinMode(AD7606_DATA_PINS[i], INPUT);
    }
    pinMode(AD7606_CONVST_PIN, OUTPUT);
    pinMode(AD7606_RESET_PIN,  OUTPUT);
    pinMode(AD7606_RD_PIN,     OUTPUT);
    pinMode(AD7606_CS0_PIN,    OUTPUT);
    pinMode(AD7606_CS1_PIN,    OUTPUT);
    pinMode(AD7606_BUSY0_PIN,  INPUT);
    pinMode(AD7606_BUSY1_PIN,  INPUT);

    // Idle levels: CS high (bus released), RD high, CONVST low, RESET inactive.
    pinLow(AD7606_CONVST_PORT, AD7606_CONVST_BIT);
    pinHigh(AD7606_RD_PORT,    AD7606_RD_BIT);
    pinHigh(AD7606_CS0_PORT,   AD7606_CS0_BIT);
    pinHigh(AD7606_CS1_PORT,   AD7606_CS1_BIT);
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

// Latch DB0..DB15 with two reads: low byte from GPIOJ, high byte from GPIOK.
inline uint16_t Adc7606Parallel::readBus() {
    const uint32_t lo = (AD7606_DB_LOW_PORT->IDR  & AD7606_DB_LOW_MASK);
    const uint32_t hi = (AD7606_DB_HIGH_PORT->IDR & AD7606_DB_HIGH_MASK);
    return (uint16_t)(lo | (hi << 8));
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

void Adc7606Parallel::readModule(uint8_t moduleIndex, int32_t* out, uint8_t base) {
    GPIO_TypeDef* csPort = (moduleIndex == 0) ? AD7606_CS0_PORT : AD7606_CS1_PORT;
    const uint32_t csBit = (moduleIndex == 0) ? AD7606_CS0_BIT : AD7606_CS1_BIT;

    pinLow(csPort, csBit);   // assert CS -> this module drives the shared bus
    for (uint8_t ch = 0; ch < AD7606_CH_PER_MODULE; ch++) {
        pinLow(AD7606_RD_PORT, AD7606_RD_BIT);        // RD low -> next channel on the bus
        tinyDelayNs(AD7606_TACC_NS);
        out[base + ch] = (int16_t)readBus();
        pinHigh(AD7606_RD_PORT, AD7606_RD_BIT);
        tinyDelayNs(AD7606_TRD_NS);
    }
    pinHigh(csPort, csBit);  // release the bus
}

void Adc7606Parallel::readAll(int32_t channels[AD7606_NUM_CHANNELS]) {
    convstPulse();           // both modules convert at the same instant
    waitReady();             // wait for both BUSY lines to fall
    readModule(0, channels, 0);
    readModule(1, channels, AD7606_CH_PER_MODULE);
}

int32_t Adc7606Parallel::readCombined() {
    int32_t channels[AD7606_NUM_CHANNELS];
    readAll(channels);
    int64_t sum = 0;
    for (uint8_t i = 0; i < AD7606_NUM_CHANNELS; i++) {
        sum += channels[i];
    }
#if SORTER_COMBINE_MEAN
    return (int32_t)(sum / AD7606_NUM_CHANNELS);
#else
    return (int32_t)sum;
#endif
}
