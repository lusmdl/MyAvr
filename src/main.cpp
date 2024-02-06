/**
 * @file    ExampleMain.cpp
 * @brief   Example program demonstrating the usage of the MyWS2812B library to control WS2812B LEDs.
 * @author  lusmdl
 * @date    05.02.2024
 * 
 * This is an example.
 */

#include <MyAvr.hpp>

/**
 * @brief MyAtmega328p - Example object of a microcontroller
 * 
 * This class represents a microcontroller of type Atmega328p.
 * It can be used to control and program the microcontroller.
 * 
 * @param F_CPU The CPU frequency of the microcontroller. Can be set in the platformio.ini 
 * with "board_build.f_cpu = 8000000UL".
 */
MyAtmega328p myAVR(F_CPU);

int main(void)
{
  
  myAVR.test(100);

  myAVR.initUart(9600);

  uint8_t matrixPins[4] {PD2, PD3, PD4, PD5};
  MyButtonMatrix2x2 btnMatrix(DDRD, PORTD, PIND, matrixPins);
  
  while (1)
  {
    /* code */
    
    myAVR.printUart("\nBEGIN\n");

    for ( int i = 0; i < 4; i++)
    {
      myAVR.printUart ("button #" + String(i) + ": " +  String(btnMatrix.getButtonStatus(i)) + "\n");
    
    }
      myAVR.delayMs(1000);
  }
  return 0;
}