#ifndef MYATMEGA328P_HPP
#define MYATMEGA328P_HPP

// HEADER

#include "MyAvr.hpp"



/**
 * @brief class for Microchip atmega 328p
 * 
 * 
 * 
 * 
 * 
 * 
*/
class MyAtmega328p : public MyController {
    
    // Klasse mit Methoden, rund um den Atmega328p von Microchip
    // µController
    
    private:

        // SETTER

        uint32_t setTC01Prescaler(enum_tcprescalers prescaler , volatile uint8_t &reg, uint8_t bit02, uint8_t bit01, uint8_t bit00);

        // SENDING

        void sendCharUart(char c);

    protected:


    public:
        
        // MEMBER

        volatile uint64_t millis_; // Zeit in millisekunden in der der Controller läuft

        // CONSTRUCTOR
        
        MyAtmega328p(unsigned long freq);

        // SETTER

        void setTC0Config(enum_tcmodes mode, enum_tcprescalers prescaler, uint8_t value_top = 255);

        // PRINTER

        void printUart(const char* str);
        void printUart(const String& str);

        // WRITER

        void writeToEEPROM(uint16_t address, uint8_t value);
        void writeToEEPROM(uint16_t address, const String& str);
        
        // READER

        uint8_t readFromEEPROM(uint16_t address);
        void readFromEEPROM(uint16_t address, String& str, size_t size_buffer);

        //INIT

        void initUart(uint32_t baudrate);

        // EXECUTER

        void execTest(uint16_t delay);
        void execSleep(enum_sleepmodes mode, uint8_t bits_power_reduction, bool enable_bod_sleep);
};
#endif