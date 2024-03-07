
// HEADER
#include "MyButtonMatrix2x2.hpp"


/**
 * @brief the object of the class MyButtonMatrix2x2 with the given parameters.
 * 
 * @param DDXn Reference to the Data Direction Register of the corresponding port
 * @param PORTXn Reference to the Output Register of the corresponding port
 * @param PINXn Reference to the Input Register of the corresponding port
 * @param bitPosition Array of bit positions for the button matrix
 * 
 * @warning This method may be unsafe in certain conditions and unexpected behavior could occur.
 * 
 * @note It is important to ensure the correct mapping of bit positions for button matrix functionality.
 * 
 * @example 
 * MyButtonMatrix2x2 matrix(PORTD, PORTD, PIND, {0, 1, 2, 3});
 */
MyButtonMatrix2x2 ::MyButtonMatrix2x2(volatile uint8_t& DDXn, volatile uint8_t& PORTXn, volatile uint8_t& PINXn, uint8_t bitPosition[4] ) : registerPtrDataDirection(&DDXn), registerPtrOutput(&PORTXn), registerPtrInput(&PINXn)  {

  for (uint8_t i = 0; i < 4; i++)
  {
    bit[i] = bitPosition[i];
  }
}


/**
 * @brief This method returns the status of a button in a 2x2 matrix.
 * 
 * @param button The index of the button
 * @warning This method may behave unpredictably in certain scenarios and is considered unsafe due to potential unspecific behavior.
 * @note Ensure proper connection of the button matrix before calling this method.
 * @example 
 * bool buttonStatus = getButtonStatus(1);
 * if(buttonStatus) {
 *   // Do something
 * } else {
 *   // Do something else
 * }
 */
bool MyButtonMatrix2x2 ::getButtonStatus(uint8_t button) {  

  uint8_t sense = button % 2;
  uint8_t ground = (button < 2) ? 2 : 3;
  uint8_t live[2] {0,0};

  uint8_t count = 0;
  for (uint8_t i = 0; i < maxBtn; i++)
  {
    if (i == sense || i == ground) {

    } else {
      live[count] = i;
      count++;  
    }
  }

  setGpioConfig(inputPullUp, *registerPtrDataDirection, *registerPtrOutput, bit[sense]);
  setGpioConfig(outputSink, *registerPtrDataDirection, *registerPtrOutput, bit[ground]);
  setGpioConfig(outputSource, *registerPtrDataDirection, *registerPtrOutput, bit[live[0]]);
  setGpioConfig(outputSource, *registerPtrDataDirection, *registerPtrOutput, bit[live[1]]);
  
  nop();

  
  return getBit(*registerPtrInput, bit[sense])? false : true;

}