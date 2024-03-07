#ifndef MYBUTTON_HPP
#define MYBUTTON_HPP

// HEADER

#include "MyAvr.hpp"


class MyButton : private MyController {

    private:

        // DATA

        volatile uint8_t* registerPtr;  // Zeiger auf das Register des Tasters
        uint8_t bit;                    // Bitposition des Tasters im Register
        bool pushed;                    // Tasterstatus (gedrückt oder nicht gedrückt)
        uint32_t numbersGetPushed;      // Anzahl der Tasterbetätigungen
        bool paraInvert;                // invert the HIGH Signal (usefull if pullup is in use)
        

    protected:


    public:

        // CONSTRUCTOR

        MyButton(volatile uint8_t& PINXn, uint8_t bitPosition, bool invertButton = false);


        // GETTER

        bool getStatus();


        // SETTER

        void setStatus(uint32_t newValue = 0);
};

#endif