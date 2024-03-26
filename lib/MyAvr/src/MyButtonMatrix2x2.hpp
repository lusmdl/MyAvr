#ifndef MYBUTTONMATRIX2X2_HPP
#define MYBUTTONMATRIX2X2_HPP

// HEADER
#include "MyAvr.hpp"


class MyButtonMatrix2x2 : private MyController {

    private:

        // MEMBER
    
        static const uint8_t BTN_MAX_ {4};

        // Zeiger auf das Register des Tasters
        volatile uint8_t* ptrDataDirectionRegister_;

        // Zeiger auf das Register des Tasters
        volatile uint8_t* ptrOutputRegister_;

        // Zeiger auf das Register des Tasters
        volatile uint8_t* ptrInputRegister_;

        // Bitposition des Tasters im Register
        uint8_t bit_ [BTN_MAX_];                    

    protected:


    public:

        // CONSTRUCTOR

        MyButtonMatrix2x2(volatile uint8_t& ddxn, volatile uint8_t& portxn, volatile uint8_t& pinxn, uint8_t bit_position[BTN_MAX_]);


        // GETTER

        bool getButtonStatus (uint8_t button);
};
#endif
