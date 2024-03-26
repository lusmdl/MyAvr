#ifndef MYBUTTON_HPP
#define MYBUTTON_HPP

// HEADER

#include "MyAvr.hpp"


class MyButton : private MyController {

    private:

        // MEMBER

        // Zeiger auf das Register des Tasters
        volatile uint8_t* ptrRegister_;

        // Bitposition des Tasters im Register
        uint8_t bit_;

        // Tasterstatus (gedrückt oder nicht gedrückt)
        bool pushed_;
        
        // Anzahl der Tasterbetätigungen
        uint32_t numberGetPushed_;
        
        // invert the HIGH Signal (usefull if pullup is in use)
        bool enableInvert_;

    protected:


    public:

        // CONSTRUCTOR
        
        MyButton(volatile uint8_t& pinxn, uint8_t bit_position, bool invert = false);


        // GETTER

        bool getStatus();


        // SETTER

        void setStatus(uint32_t value_new = 0);
};

#endif