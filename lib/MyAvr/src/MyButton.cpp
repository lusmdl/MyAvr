
// HEADER
#include "MyButton.hpp"

/**
 * @brief Constructor of the class MyButton
 * Initializes the member variables of the class.
 * 
 * @param PINXn A reference to the register containing the status of the button.
 * @param bitPosition The position of the bit in the register representing the button status.
 * @param invertButton Indicates if the button input should be inverted.
 * 
 * @warning This method may have unforeseen consequences and could be insecure at certain points.
 * 
 * @note Make sure to properly configure the parameters to avoid unexpected behaviors.
 * 
 * @example 
 * MyButton button(PINXn, 3, true);
 */
MyButton ::MyButton(volatile uint8_t& PINXn, uint8_t bitPosition, bool invertButton) : registerPtr(&PINXn), bit(bitPosition), pushed(false), numbersGetPushed(0), paraInvert(invertButton) {}


/**
 * @brief Returns the status of the button.
 * Checks the state of the button and updates the count of pressed keys.
 * 
 * @param registerPtr Pointer to the register for button status checking
 * @param bit The bit position of the button in the register
 * @param paraInvert Flag for inverting the button status
 * @param numbersGetPushed Number of times the button has been pressed
 * @param pushed Flag indicating if the button is currently pressed
 * 
 * @return The status of the button (true if the button is pressed, false otherwise)
 * 
 * @warning This method may have unexpected behavior and is considered unsafe in certain scenarios.
 * 
 * @note Use the parameter paraInvert carefully to handle inverted button status.
 * 
 * @example 
 * bool buttonStatus = getStatus(&register, 3, true, numbersPressed, buttonPressed);
 * if (buttonStatus) {
 *    // Do something if the button is pressed
 * }
 */
bool MyButton ::getStatus() {

  /* Führe eine NOP-Operation für die Synchronisation aus */
  nop();

  bool buttonStatus = getBit(*registerPtr, bit);

  
  if (paraInvert) {
    /*
    if the parameter is set, swap the bit
    this is usefull if you use pullup resistors
    */
    buttonStatus = !buttonStatus;
  }

  if (buttonStatus) {
    ++numbersGetPushed;
    pushed = true;
  } else {
    pushed = false;
  }
  return buttonStatus;
}


/**
 * @brief Sets the status of the button.
 * 
 * @param newValue The new value for the amount of button numbers pushed (default value is 0)
 * 
 * @warning This method may be unsafe in certain scenarios where unexpected behavior could occur.
 * 
 * @note Make sure to handle corner cases carefully when using this method.
 * 
 * @example
 * MyButton myButton;
 * myButton.setStatus(3);
 * // This sets the button status with 3 pushed numbers
 */
void MyButton ::setStatus(uint32_t newValue) {
  numbersGetPushed = newValue;
  if (newValue == 0) {
    pushed = false;
  }
}
