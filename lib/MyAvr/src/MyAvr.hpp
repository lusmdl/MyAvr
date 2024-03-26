#ifndef MYAVR_HPP
#define MYAVR_HPP


// MACROS

#define BITMASK(bit) (1 << (bit))

// AVR 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>


// C/C++

#include <WString.h>

// OOP
#include <MyBehavioralDesignPatterns.hpp>


// ENUMS


/**
 * 
 * @brief Konfiguration der General Porpus IOs
 * Legt fest, ob es sich um einen Eingang oder Ausgang handelt
 * 
 * 
 * 
*/
enum enum_gpiomodes {

    INPUT_TRI_STATE_1,
    INPUT_PULLUP,
    INPUT_TRI_STATE_2,
    OUTPUT_SINK,
    OUTPUT_SOURCE
};

/**
 * @brief Konfigurationsmodi für Timer
 * 
 * 
*/
enum enum_tcmodes {

    NORMAL,
    PWM_PHASE_CORRECT,
    CTC,
    FAST_PWM,
    PWM_PHASE_CORRECT_COMPARE_A,
    FAST_PWM_COMPARE_A
};

/**
 * 
 * @brief Timer/Counter Prescalers
 * 
 * 
 * @note
 * NO_CLOCK_SOURCE,
    CLK_1,
    CLK_8,
    CLK_32,
    CLK_64,
    CLK_128,
    CLK_256,
    CLK_1024,
    EXT_CLK_SOURCE_RISING_EDGE,
    EXT_CLK_SOURCE_FALLING_EDGE
 * 
 * 
 * 
 * 
 * 
 * 
*/
enum enum_tcprescalers {

    NO_CLOCK_SOURCE,
    CLK_1,
    CLK_8,
    CLK_32,
    CLK_64,
    CLK_128,
    CLK_256,
    CLK_1024,
    EXT_CLK_SOURCE_RISING_EDGE,
    EXT_CLK_SOURCE_FALLING_EDGE
};

/**
 * @brief Sleep Modes
 * 
 * 
 * @note
 * IDLE,
    ADC_NOISE_REDUCTION,
    POWER_DOWN,
    POWER_SAVE,
    STANDBY,
    EXTENDED_STANDBY
 * 
 * 
 * 
 * 
 * 
 * 
 * 
*/
enum enum_sleepmodes {

    // data

    IDLE,
    ADC_NOISE_REDUCTION,
    POWER_DOWN,
    POWER_SAVE,
    STANDBY,
    EXTENDED_STANDBY

};


// CLASSES

// BASIC CLASSES

class MyController {

    private:


    protected:

        // DATA 

        //  CPU Frequenz - Atmega328p standard = 8 Mhz -> 8000000
        unsigned long cpuFreq_; 

    public:

        // GETTER

        bool getBit(volatile uint8_t &reg, uint8_t bit);
        
        // SETTER

        void setBit(volatile uint8_t &reg, uint8_t bit, bool value = true);
        void setBitMask(volatile uint8_t &reg, uint8_t bitMask, bool value = true);
        void setGpioConfig(enum_gpiomodes mode, volatile uint8_t &ddxn, volatile uint8_t &portxn, uint8_t bit);
        
        // EXECUTIONS

        void execNop();
        void execDelayUs(uint16_t us);
        void execDelayMs(uint32_t ms);

        // CLEAR

        void clearBit(volatile uint8_t &reg, uint8_t bit);
        void clearBitMask(volatile uint8_t &reg, uint8_t bit_mask);
};


// µController

// ATMEGAs

// ATMEGA328p
#include "Atmega328pISR.h"
#include "MyAtmega328p.hpp"


// PERIPHERALS

// BUTTONS

#include "MyButton.hpp"
#include "MyButtonMatrix2x2.hpp"


#endif