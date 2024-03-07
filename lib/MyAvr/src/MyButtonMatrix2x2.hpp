#ifndef MYBUTTONMATRIX2X2_HPP
#define MYBUTTONMATRIX2X2_HPP

// HEADER
#include "MyAvr.hpp"


class MyButtonMatrix2x2 : private MyController {

    private:

        // DATA
    
        static const uint8_t maxBtn {4};
        volatile uint8_t* registerPtrDataDirection;  // Zeiger auf das Register des Tasters
        volatile uint8_t* registerPtrOutput;  // Zeiger auf das Register des Tasters
        volatile uint8_t* registerPtrInput;  // Zeiger auf das Register des Tasters
        uint8_t bit [maxBtn];                    // Bitposition des Tasters im Register


    protected:


    public:

        // CONSTRUCTOR
        MyButtonMatrix2x2(volatile uint8_t& DDXn, volatile uint8_t& PORTXn, volatile uint8_t& PINXn, uint8_t bitPosition[maxBtn]);


        // GETTER

        bool getButtonStatus (uint8_t button);
};

#endif
