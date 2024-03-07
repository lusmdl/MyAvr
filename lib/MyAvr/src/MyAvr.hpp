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


// ENUMS

/*
Konfiguration der General Porpus IOs
Legt fest, ob es sich um einen Eingang oder Ausgang handelt
*/
enum gpioMode {

    inputTriState1,
    inputPullUp,
    inputTriState2,
    outputSink,
    outputSource
};

/*
Konfigurationsmodi für Timer
*/
enum tcModes {

    normal,
    pwmPhaseCorrect,
    ctc,
    fastPwm,
    pwmPhaseCorrectCompareA,
    fastPwmCompareA
};

/*
Vorteiler für Takt
*/
enum tcPrescalers {

    noClockSource,
    clk1,
    clk8,
    clk32,
    clk64,
    clk128,
    clk256,
    clk1024,
    extClkSourceRisingEdge,
    extClkSourceFallingEdge
};

/*
Sleep Modes
@note
Idle,  
ADC Noise Reduction, 
Power-down,
Power-save, 
Standby, 
Extended Standby
*/
enum sleepModes {

    // data

    idle,
    adcNoiseReduction,
    powerDown,
    powerSave,
    Standby,
    extendedStandby

};


// CLASSES


// BASIC CLASSES

class MyController {

    private:


    protected:

        // DATA 

        unsigned long cpuFreq; //  CPU Frequenz - Atmega328p standard = 8 Mhz -> 8000000UL


        // METHODS


    public:

        // METHODS

        void nop();
        void delayUs(uint16_t us);
        void delayMs(uint32_t ms);


        // GETTER

        bool getBit(volatile uint8_t &reg, uint8_t bit);


        // SETTER

        void setBit(volatile uint8_t &reg, uint8_t bit, bool value = true);
        void clearBit(volatile uint8_t &reg, uint8_t bit);
        void setBitMask(volatile uint8_t &reg, uint8_t bitMask, bool value = true);
        void clearBitMask(volatile uint8_t &reg, uint8_t bitMask);
        void setGpioConfig(gpioMode mode, volatile uint8_t &DDxn, volatile uint8_t &PORTxn, uint8_t bit);
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