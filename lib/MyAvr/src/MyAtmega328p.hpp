#ifndef MYATMEGA328P_HPP
#define MYATMEGA328P_HPP

// HEADER

#include "MyAvr.hpp"



// ATMEGA

/*
Klasse mit Methoden, rund um den Atmega328p von Microchip
*/
class MyAtmega328p : public MyController {
    
    // µController
    
    private:

        // METHODS

        void putcharUart(char c);


        // SETTER

        uint32_t setTC01Prescaler(tcPrescalers prescaler , volatile uint8_t &reg, uint8_t bit02, uint8_t bit01, uint8_t bit00);


    protected:


    public:
        
        /* DATA */

        volatile uint64_t millis; // Zeit in millisekunden in der der Controller läuft


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
        void setTC0Config(tcModes mode, tcPrescalers prescaler, uint8_t topValue = 255);
};

#endif