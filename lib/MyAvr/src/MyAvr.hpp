#ifndef MYAVR_HPP
#define MYAVR_HPP


/* AVR */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>


/* MY */
#include <Atmega328pISR.h>


/* C/C++ */
#include <WString.h>


// MACROS

#define BITMASK(bit) (1 << (bit))


// USER DEFINED TYPES

typedef uint64_t time;


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
enum sleepModes
{
    /* data */
    idle,
    adcNoiseReduction,
    powerDown,
    powerSave,
    Standby,
    extendedStandby

};


/* CLASSES */


/* BASIS CLASSES */

class MyController {

    private:


    protected:

        /* DATA */ 

        unsigned long cpuFreq; //  CPU Frequenz - Atmega328p standard = 8 Mhz -> 8000000UL


        /* METHODS */


    public:

        /* METHODS */

        void nop();
        void delayUs(uint16_t us);
        void delayMs(uint32_t ms);


        /* GETTER */

        bool getBit(volatile uint8_t &reg, uint8_t bit);


        /* SETTER */

        void setBit(volatile uint8_t &reg, uint8_t bit, bool value = true);
        void clearBit(volatile uint8_t &reg, uint8_t bit);
        void setBitMask(volatile uint8_t &reg, uint8_t bitMask, bool value = true);
        void clearBitMask(volatile uint8_t &reg, uint8_t bitMask);
        void setGpioConfig(gpioMode mode, volatile uint8_t &DDxn, volatile uint8_t &PORTxn, uint8_t bit);
};


/* µController */


/* ATMEGA */

/*
Klasse mit Methoden, rund um den Atmega328p von Microchip
*/
class MyAtmega328p : public MyController {
    
    private:

        /* METHODS */

        void putcharUart(char c);


        /* GETTER */


        /* SETTER */

        uint32_t setTC01Prescaler(tcPrescalers prescaler , volatile uint8_t &reg, uint8_t bit02, uint8_t bit01, uint8_t bit00);


    protected:


    public:
        
        /* DATA */

        volatile time millis; // Zeit in millisekunden in der der Controller läuft


        /* CONSTRUCTOR */

        MyAtmega328p(unsigned long freq);
        

        /* METHODS */

        void test(uint16_t delay);
        void initUart(uint32_t baudrate);
        void sleep(sleepModes sleepMode, uint8_t powerReductionBits, bool enableBODSleep);


        /* GETTER */

        uint8_t readFromEEPROM(uint16_t address);
        void readFromEEPROM(uint16_t address, String& str, size_t bufferSize);


        /* SETTER */

        void printUart(const char* str);
        void printUart(const String& str);
        void writeToEEPROM(uint16_t address, uint8_t value);
        void writeToEEPROM(uint16_t address, const String& str);
        void setTC0Config(tcModes mode, tcPrescalers prescaler, time topTime = 1);
};


/* PERIPHERALS */

class MyButton : private MyController {

    private:

        /* DATA */

        volatile uint8_t* registerPtr;  // Zeiger auf das Register des Tasters
        uint8_t bit;                    // Bitposition des Tasters im Register
        bool pushed;                    // Tasterstatus (gedrückt oder nicht gedrückt)
        uint32_t numbersGetPushed;      // Anzahl der Tasterbetätigungen
        bool paraInvert;                // invert the HIGH Signal (usefull if pullup is in use)
        

    protected:


    public:

        /* CONSTRUCTOR */

        MyButton(volatile uint8_t& PINXn, uint8_t bitPosition, bool invertButton = false);


        /* GETTER */

        bool getStatus();


        /* SETTER */ 

        void setStatus(uint32_t newValue = 0);
};


class MyButtonMatrix2x2 : private MyController {

    private:

        /* DATA */
    
        static const uint8_t maxBtn {4};
        volatile uint8_t* registerPtrDataDirection;  // Zeiger auf das Register des Tasters
        volatile uint8_t* registerPtrOutput;  // Zeiger auf das Register des Tasters
        volatile uint8_t* registerPtrInput;  // Zeiger auf das Register des Tasters
        uint8_t bit [maxBtn];                    // Bitposition des Tasters im Register


    protected:


    public:

        /* CONSTRUCTOR */

        MyButtonMatrix2x2(volatile uint8_t& DDXn, volatile uint8_t& PORTXn, volatile uint8_t& PINXn, uint8_t bitPosition[maxBtn]);


        /* GETTER */

        bool getButtonStatus (uint8_t button);
};


#endif
