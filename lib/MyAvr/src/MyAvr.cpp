
// HEADER
#include "MyAvr.hpp"


/**
 * @brief Retrieves the value of a specific bit in a register.
 * This function reads and returns the value of a specified bit in a register.
 * 
 * @param reg The register from which the bit value is to be read
 * @param bit The position of the bit in the register (0 - 7)
 * 
 * @return The value of the specified bit (true if set, false if not set)
 * 
 * @warning This method may be sensitive to changes in hardware configuration or external interference. Use with caution in critical systems.
 * 
 * @note The bit position should be within the range of 0 to 7 for proper functionality.
 * 
 * @example 
 * uint8_t myRegister = 0b10101010;
 * bool bitValue = myController.getBit(myRegister, 3);
 * // The value of bitValue will be true (1) as the bit at position 3 is set in myRegister
 */
bool MyController::getBit(volatile uint8_t &reg, uint8_t bit) {

  return ((reg & (1 << bit)) != 0);
}


/**
 * @brief Sets a specific bit in a register to a desired value.
 * 
 * @param reg The register where the bit will be set
 * @param bit The position of the bit in the register
 * @param value The value to set the bit to (true for 1, false for 0)
 * 
 * @warning This method may not be safe in all contexts as it directly manipulates memory registers.
 * 
 * @note This method uses bitwise operations to set or clear the specified bit in the given register.
 * 
 * @example 
 * MyController myController;
 * volatile uint8_t myRegister = 0b00001010;
 * myController.setBit(myRegister, 2, true);
 */
void MyController::setBit(volatile uint8_t &reg, uint8_t bit, bool value) {

  if (value) {

    reg |= (1 << bit); // set the bit to 1
  }
  else {
    reg &= ~(1 << bit); // set the bit to 0
  }
}


/**
 * @brief Clears a specific bit in a given register.
 * 
 * @param reg The register in which the bit will be cleared.
 * @param bit The bit position to be cleared.
 * @return void
 * 
 * @note This function sets the specified bit in the register to 0.
 * @warning This method may not be safe if the register is shared among multiple threads.
 * 
 * @example
 * volatile uint8_t myRegister = 0b10101010; // Binary representation
 * clearBit(myRegister, 3);
 * // After calling clearBit, myRegister will be 0b10100010
 */
void MyController::clearBit(volatile uint8_t &reg, uint8_t bit) {
  
  reg &= ~(1 << bit); // set the bit to 0
}


/**
 * @brief Sets or clears the bitmask in a specified register based on the given value.
 * 
 * @param reg The register in which the bitmask should be set or cleared.
 * @param bitMask The bitmask to set or clear.
 * @param value The value specifying whether the bitmask should be set or cleared.
 * 
 * @remarks This method sets the bit to 1 or 0 in the specified register, depending on the value parameter.
 * 
 * @note This method assumes that the register is declared as volatile and the bitmask is passed in the correct format.
 * 
 * @warning This method may lead to unexpected behavior if the bitmask is incorrect or the register is not properly defined.
 * 
 * @example
 * MyController controller;
 * volatile uint8_t myRegister = 0x00;
 * 
 * controller.setBitMask(myRegister, 0b00000010, true);
 * // After this call, myRegister will have the bit at position 1 set to 1
 */
void MyController::setBitMask(volatile uint8_t &reg, uint8_t bitMask, bool value) {

  if (value) {

    reg |= bitMask; // set the bit to 1
  } 
  else {

    reg &= ~bitMask; // set the bit to 0
  }
}


/**
 * Clears the bitmask in a specified register.
 * 
 * @param reg The register in which the bitmask should be cleared.
 * @param bitMask The bitmask to be cleared.
 * 
 * @remarks This method clears the bit to 0 in the specified register by inverting the bitmask and performing a bitwise AND operation.
 * 
 * @note This method assumes that the register is declared as volatile and the bitmask is passed in the correct format.
 * 
 * @warning This method may lead to unpredictable behavior if the bitmask is incorrect or the register is not correctly defined.
 * 
 * @example 
 * MyController mc;
 * volatile uint8_t reg = 0b10101100;
 * uint8_t mask = 0b00101010;
 * mc.clearBitMask(reg, mask);
 */
void MyController::clearBitMask(volatile uint8_t &reg, uint8_t bitMask) {

  reg &= ~bitMask; // set the bit to 0
}


/**
 * @brief Sets the GPIO configuration based on the selected mode.
 * 
 * @param mode The desired GPIO mode
 * @param DDxn The corresponding Data Direction Register
 * @param PORTxn The corresponding Port Register
 * @param bit The position of the bit in the register
 * 
 * @warning This method may have unexpected behavior and is considered unsafe in certain cases.
 * 
 * @note Make sure to handle memory access carefully when using this method.
 * 
 * @example 
 * MyController myController;
 * myController.setGpioConfig(inputPullUp, DDRA, PORTA, 5);
 */
void MyController ::setGpioConfig(enum_gpiomodes mode, volatile uint8_t &DDxn, volatile uint8_t &PORTxn, uint8_t bit)
{

  switch (mode) {

    case INPUT_TRI_STATE_1:

      setBit(DDxn, bit, 0);
      setBit(PORTxn, bit, 0);
      break;

    case INPUT_PULLUP:

      setBit(DDxn, bit, 0);
      setBit(PORTxn, bit, 1);
      setBit(MCUCR, PUD, 0);
      break;

    case INPUT_TRI_STATE_2:

      setBit(DDxn, bit, 0);
      setBit(PORTxn, bit, 1);
      setBit(MCUCR, PUD, 1);
      break;

    case OUTPUT_SINK:

      setBit(DDxn, bit, 1);
      setBit(PORTxn, bit, 0);
      break;

    case OUTPUT_SOURCE:

      setBit(DDxn, bit, 1);
      setBit(PORTxn, bit, 1);
      break;

    default:

      break;
  }
}


/**
 * @brief Delays the execution for the specified number of milliseconds.
 * 
 * @param ms The number of milliseconds to delay
 * @warning This method may have unexpected behavior and is potentially insecure in certain aspects.
 * @note The delay is achieved by using a loop to delay for 65535 microseconds before handling the remaining time.
 * @example 
 * MyController myController;
 * myController.delayMs(1000);
 */
void MyController::execDelayMs(uint32_t ms) {

    // Anzahl der Mikrosekunden basierend auf den Millisekunden berechnen

    const uint32_t sizeOf32Bit = 0xFFFFFFFF;
    uint32_t us {sizeOf32Bit};

    // Prüfe ob zu viele millisekunden ausgewählt wurden

    uint32_t maxMillis = sizeOf32Bit/1000;
    if (ms < (maxMillis)) {

      us = ms * 1000;
    } 

    // Anzahl der 65535-Mikrosekunden-Pakete berechnen

    const uint16_t sizeOf16Bit {0xFFFF};
    uint8_t numberOfLoops = (us / sizeOf16Bit);

    // Verzögere für jedes 65535-Mikrosekunden-Paket

    for (uint8_t loops = 0; loops < numberOfLoops; loops++) {
      
      execDelayUs(sizeOf16Bit);
    }

    // Verzögere den verbleibenden Teil
    execDelayUs(us % sizeOf16Bit);
}


/**
 * @brief Delay the execution for a specified number of microseconds using inline assembly code
 * @param microseconds The number of microseconds to delay the execution for
 * @param cpuFrequency The CPU frequency in Hz
 * @warning This method relies on low-level inline assembly code and may have undefined behavior depending on the hardware and compiler optimizations
 * @note Ensure the accuracy of the CPU frequency parameter to achieve the desired delay
 * @example
 * MyController myCtrl;
 * myCtrl.delayUs(100); // Delay execution for 100 microseconds
 */
void MyController::execDelayUs(uint16_t us) {

    // Anzahl der Iterationen berechnen, basierend auf der CPU-Frequenz

    const uint8_t numberOfClkPerNOP {8};
    uint32_t iterations = ((cpuFreq_ / 1000000) * us / numberOfClkPerNOP);

    // Delay durchführen
    asm volatile (
        "1:          \n"
        "nop         \n"
        "subi %A0, 1 \n"
        "sbci %B0, 0 \n"
        "sbci %C0, 0 \n"
        "sbci %D0, 0 \n"
        "brne 1b     \n"
        : "=w" (iterations)
        : "0" (iterations)
    );
}


/**
 * @brief Executes a NOP (No Operation) command.
 * 
 * @warning This method performs a no operation command which may lead to potential 
 * unexpected behavior and the method may be unreliable in some cases.
 * 
 * @note NOP command is used for executing an idle function and can be used for synchronization or delays.
 * 
 * @example 
 * MyController myController;
 * myController.nop();
 */
void MyController ::execNop() {

  asm volatile("nop");
}