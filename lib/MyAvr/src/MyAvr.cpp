
#include "MyAvr.hpp"





/**
 * @brief Constructor of the MyAtmega328p class
 * Initializes the CPU frequency of the microcontroller.
 * 
 * @param freq The desired CPU frequency of the microcontroller
 * 
 * @warning This method is considered unsafe in situations where incorrect CPU frequency can lead to system instability.
 * 
 * @note It is important to verify the compatibility of the specified frequency with the microcontroller specifications.
 * 
 * @example 
 * MyAtmega328p myController(F_CPU);
 */
MyAtmega328p ::MyAtmega328p(unsigned long freq) {

  cpuFreq = freq;
}








/**
 * @brief Writes a string to the EEPROM memory.
 * 
 * @param address The address in the EEPROM where the string will be written
 * @param str The string to be written
 * 
 * @warning Access to EEPROM memory may overwrite existing data if not carefully managed.
 * @note The length of the string should not exceed the available space in EEPROM.
 * 
 * @example 
 * MyAtmega328p myController;
 * myController.writeToEEPROM(0, "Hello World");
 */

void MyAtmega328p ::writeToEEPROM(uint16_t address, const String& str) {
  // Speicherzugriff einschränken auf den EEPROM-Bereich
  if (address >= 0 && address < E2END) {
    size_t length = str.length();
    if (length > E2END - address) {
      length = E2END - address;
    }
    eeprom_write_block(reinterpret_cast<const void*>(str.c_str()), reinterpret_cast<void*>(address), length);
  }
}












/**
 * @brief Reads a string from the EEPROM at a given address into a buffer.
 * 
 * @param address The address in the EEPROM from which the string should be read
 * @param str A reference to a string where the read string will be stored
 * @param bufferSize The size of the buffer for the read string
 * 
 * @warning 
 * This method may produce unexpected results if the address is outside the valid EEPROM range.
 * 
 * @note 
 * It is important to ensure that the buffer size is sufficient for the read string length.
 * 
 * @example 
 * String myString;
 * MyAtmega328p myController;
 * myController.readFromEEPROM(100, myString, 20);
 */
void MyAtmega328p ::readFromEEPROM(uint16_t address, String& str, size_t bufferSize) {
  // Überprüfen, ob Adresse innerhalb des EEPROM-Bereichs liegt
  if (address >= 0 && address < E2END) {
    size_t length = E2END - address;
    if (length > bufferSize) {
      length = bufferSize;
    }
    char* buffer = new char[length + 1];
    eeprom_read_block(reinterpret_cast<void*>(buffer), reinterpret_cast<const void*>(address), length);
    buffer[length] = '\0';
    str = buffer;
    delete[] buffer;
  }
}















/**
 * @brief Writes a byte into the EEPROM memory at the specified address.
 * 
 * @param address The memory address in the EEPROM where the byte will be written
 * @param value The byte value to be written
 * 
 * @warning This method may not handle out-of-bounds access to the EEPROM memory.
 * 
 * @note Make sure that the address provided falls within the valid range of the EEPROM memory.
 * 
 * @example 
 * myController.writeToEEPROM(0x10, 0xAB);
 */
void MyAtmega328p ::writeToEEPROM(uint16_t address, uint8_t value) {
  // Speicher-Zugriff einschränken auf Adressebereich des EEPROM
  if (address >= 0 && address < E2END) {
    eeprom_write_byte((uint8_t*)address, value);
  }
}
  




/**
 * @brief Reads a byte from the EEPROM at the specified address.
 * 
 * @param address The address in the EEPROM from which to read the byte.
 *        Must be within the valid range of the EEPROM.
 * 
 * @warning This method may behave unpredictably if the address provided is outside the valid range of the EEPROM.
 * 
 * @return The byte read from the EEPROM at the specified address. 
 *         Returns 0 if the address is invalid.
 * 
 * @note This method uses low-level EEPROM read function, ensure correct address range is provided.
 * 
 * @example 
 * uint8_t data = myController.readFromEEPROM(100);
 * Serial.print(data); // Output: Value at address 100 in EEPROM
 */
uint8_t MyAtmega328p ::readFromEEPROM(uint16_t address) {
  // Überprüfen, ob Adresse innerhalb des EEPROM-Bereichs liegt
  if (address >= 0 && address < E2END) {
    return eeprom_read_byte((uint8_t*)address);
  }
  
  return 0; // Rückgabewert, wenn Adresse ungültig ist
}











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
bool MyController::getBit(volatile uint8_t &reg, uint8_t bit)
{
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
  if (value)
  {
    reg |= (1 << bit); // set the bit to 1
  }
  else
  {
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
  if (value)
  {
    reg |= bitMask; // set the bit to 1
  }
  else
  {
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
void MyController ::setGpioConfig(gpioMode mode, volatile uint8_t &DDxn, volatile uint8_t &PORTxn, uint8_t bit)
{

  switch (mode)
  {
  case inputTriState1:
    setBit(DDxn, bit, 0);
    setBit(PORTxn, bit, 0);
    break;

  case inputPullUp:
    setBit(DDxn, bit, 0);
    setBit(PORTxn, bit, 1);
    setBit(MCUCR, PUD, 0);
    break;

  case inputTriState2:
    setBit(DDxn, bit, 0);
    setBit(PORTxn, bit, 1);
    setBit(MCUCR, PUD, 1);
    break;

  case outputSink:
    setBit(DDxn, bit, 1);
    setBit(PORTxn, bit, 0);
    break;

  case outputSource:
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
void MyController::delayMs(uint32_t ms) {
    // Anzahl der Mikrosekunden basierend auf den Millisekunden berechnen
    const uint32_t sizeOf32Bit = 0xFFFFFFFF;
    uint32_t us {sizeOf32Bit};

    // Prüfe ob zu viele millisekunden ausgewählt wurden
    uint32_t maxMillis = sizeOf32Bit/1000;
    if (ms < (maxMillis) ){

      us = ms * 1000;
      
    } 

    // Anzahl der 65535-Mikrosekunden-Pakete berechnen
    const uint16_t sizeOf16Bit {0xFFFF};
    uint8_t numberOfLoops = (us / sizeOf16Bit);

    // Verzögere für jedes 65535-Mikrosekunden-Paket
    for (uint8_t loops = 0; loops < numberOfLoops; loops++) {
      delayUs(sizeOf16Bit);
    }

    // Verzögere den verbleibenden Teil
    delayUs(us % sizeOf16Bit);
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
void MyController::delayUs(uint16_t us) {
    // Anzahl der Iterationen berechnen, basierend auf der CPU-Frequenz
    const uint8_t numberOfClkPerNOP {8};
    uint32_t iterations = ((cpuFreq / 1000000UL) * us / numberOfClkPerNOP);

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
 * @brief Tests the functionality of the microcontroller LED by turning it on and off for a specified duration.
 * 
 * @param delay The time to wait in milliseconds between turning the LED on and off
 * @warning This method may be unsafe in certain situations and unexpected behavior could occur.
 * @note Ensure the digital pin configuration and LED setup are correctly handled before calling this method.
 * @example 
 * MyAtmega328p myDevice;
 * myDevice.test(1000); // Turn the LED on and off with a delay of 1000ms
 */
void MyAtmega328p ::test(uint16_t delay)
{
  // Den digitalen Pin als Ausgang konfigurieren
  // DDRB |= (1 << DDB5);
  setBit(DDRB, DDB5, true);

  // Die LED einschalten
  // PORTB |= (1 << PORTB5);
  setBit(PORTB, DDB5, true);
  delayMs(delay); // Eine Sekunde warten

  // Die LED ausschalten
  // PORTB &= ~(1 << PORTB5);
  setBit(PORTB, DDB5, false);
  delayMs(delay); // Eine Sekunde warten
}










/**
 * @brief Initializes UART communication on the MyAtmega328p.
 * Configures the baud rate, frame format settings, and outputs a "Hello World!" message via UART.
 * 
 * @param baudrate The desired baud rate for UART communication
 * 
 * @warning This method may encounter unforeseen issues and is considered unsafe in certain aspects.
 * 
 * @note It is important to configure the baud rate and frame format correctly to ensure proper communication.
 * 
 * @example 
 * MyAtmega328p myDevice;
 * myDevice.initUart(9600);
 */
void MyAtmega328p ::initUart(uint32_t baudrate)
{

  uint16_t ubrr = cpuFreq / (16 * baudrate) - 1;

  // Set baud rate
  UBRR0H = (uint8_t)(ubrr >> 8);
  UBRR0L = (uint8_t)ubrr;

  // Enable receiver and transmitter
  UCSR0B = (1 << RXEN0) | (1 << TXEN0);

  // Set frame format: 8data, 2stop bit
  // UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
  UCSR0C = (1 << USBS0) | (3 << UCSZ00);

  // Hello World Nachricht
  printUart("\nHello World!\n");
}










/**
 * @brief Sends a character to the UART output buffer.
 * 
 * @param c The character to be sent
 * 
 * @warning This function blocks until the output buffer is ready to accept a character.
 * 
 * @note Make sure the UART hardware interface is properly configured before calling this function.
 * 
 * @example 
 * MyAtmega328p myAtmega;
 * myAtmega.putcharUart('A');
 */
void MyAtmega328p ::putcharUart(char c)
{
  while (!(UCSR0A & (1 << UDRE0)))
    ;
  UDR0 = c;
}















/**
 * @brief Prints a string parameter over the UART.
 * 
 * @param str The string to be transmitted 
 * 
 * @warning This method may be unsafe in certain conditions.
 * 
 * @note Make sure the UART communication is properly configured before calling this method.
 * 
 * @example 
 * MyAtmega328p.atmega328p.printUart("Hello, world!");
 */
void MyAtmega328p ::printUart(const char *str)
{
  const size_t len = strlen(str);
  for (size_t i = 0; i < len; i++)
  {
    putcharUart(str[i]);
  }
}













/**
 * @brief Prints a string over the UART interface.
 * 
 * @param str The string to be printed
 * 
 * @warning This method may be unsafe in certain aspects and unexpected behavior can occur.
 * 
 * @note Make sure the UART communication is properly configured before calling this method.
 * 
 * @example 
 * MyAtmega328p atmegaDevice;
 * atmegaDevice.printUart("Hello, World!");
 */
void MyAtmega328p::printUart(const String &str)
{
  const size_t len = str.length();
  for (size_t i = 0; i < len; i++)
  {
    putcharUart(str.charAt(i));
  }
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
void MyController ::nop()
{
  asm volatile("nop");
}



/**
 * @brief Configures Timer/Counter 0 (TC0) on the Atmega328p.
 * 
 * @param mode The desired mode for TC0 (e.g. ctc - Clear Timer on Compare Match)
 * @param prescaler The desired prescaler for TC0
 * @param topValue The desired comparison value for CTC mode
 * 
 * @warning Perform the calculation of time in the main code.
 * 
 * @note Refer to the table (in the MyAvr.cpp file) for setting Timer/Counter Mode of Operation and corresponding values.
 * 
 * @example
 * MyAtmega328p.setTC0Config(MyAtmega328p::tcModes::ctc, MyAtmega328p::tcPrescalers::prescalerValue, 100);
 */
void MyAtmega328p ::setTC0Config(tcModes mode, tcPrescalers prescaler, uint8_t topValue)
{

  /*
  Konfiguriere Timer/Counter 0
  -
  | Mode  | WGM02 | WGM01 | WGM00 | Timer/Counter Mode of Operation | TOP   | Update of OCR0x at TOV Flag Set on
  | 0     | 0     | 0     | 0     | Normal                          | 0xFF  | Immediate MAX
  | 1     | 0     | 0     | 1     | PWM, Phase Correct              | 0xFF  | TOP BOTTOM
  | 2     | 0     | 1     | 0     | CTC                             | OCRA  | Immediate MAX
  | 3     | 0     | 1     | 1     | Fast PWM                        | 0xFF  | BOTTOM MAX
  | 5     | 1     | 0     | 1     | PWM, Phase Correct              | OCRA  | TOP BOTTOM
  | 7     | 1     | 1     | 1     | Fast PWM                        | OCRA  | BOTTOM TOP
  */
  switch (mode)
  {
  case ctc:

    // ctc modus einrichten
    setBit(TCCR0A, WGM00, 0);
    setBit(TCCR0A, WGM01, 1);
    setBit(TCCR0B, WGM02, 0);

    // prescaler
    auto N = setTC01Prescaler(prescaler, TCCR0B, CS02, CS01, CS00);


    // Setze den Vergleichswert für den CTC-Modus
    /*
    don't use this calculation. Do this in the main code instead!
    auto clkPerMillisekond { ( cpuFreq / N ) / 1000 };
    OCR0A = ( ( clkPerMillisekond * topTime ) - 1 );

    */
    OCR0A = topValue;

    // interrupt aktivieren
    setBit(TIMSK0, OCIE0A);
    sei();

    

    break;

  default:
    break;
  }

}























/**
 * @brief Sets the prescaler for Timer/Counter0 to a specific value.
 * 
 * @param prescaler The desired prescaler value
 * @param reg The register where the bits for the prescaler will be set
 * @param bit02 The position of Bit02 in the register
 * @param bit01 The position of Bit01 in the register
 * @param bit00 The position of Bit00 in the register
 * 
 * @return The set prescaler value (1, 8, 64, 256, 1024, or 0 for external clock source)
 * 
 * @warning This method may be unsafe in some situations where unexpected behavior could occur.
 * 
 * @note The prescaler values correspond to the clock frequency division ratios as shown in the Prescaler Table.
 * 
 * @example 
 * uint8_t prescalerReg = 0x00; // Initialize the register for prescaler settings
 * uint32_t prescalerValue = setTC01Prescaler(clk64, prescalerReg, 1, 2, 0);
 * // This sets the prescaler to 64 and updates the register accordingly
 */
uint32_t MyAtmega328p ::setTC01Prescaler(tcPrescalers prescaler, volatile uint8_t &reg, uint8_t bit02, uint8_t bit01, uint8_t bit00)
{

  /*
  Prescaler Tabelle
  -
  |02 |01 |00 |
  | 0 | 0 | 0 | No clock source (Timer/Counter stopped)
  | 0 | 0 | 1 | clkI/O/1 (No prescaling)
  | 0 | 1 | 0 | clkI/O/8 (From prescaler)
  | 0 | 1 | 1 | clkI/O/64 (From prescaler)
  | 1 | 0 | 0 | clkI/O/256 (From prescaler)
  | 1 | 0 | 1 | clkI/O/1024 (From prescaler)
  | 1 | 1 | 0 | External clock source on T0 pin. Clock on falling edge.
  | 1 | 1 | 1 | External clock source on T0 pin. Clock on rising edge.
  */
  switch (prescaler)
  {
  case clk1:
    setBit(reg, bit00, true);
    setBit(reg, bit01, false);
    setBit(reg, bit02, false);
    return 1;
    break;

  case clk8:
    setBit(reg, bit00, false);
    setBit(reg, bit01, true);
    setBit(reg, bit02, false);
    return 8;
    break;

  case clk64:
    setBit(reg, bit00, true);
    setBit(reg, bit01, true);
    setBit(reg, bit02, false);
    return 64;
    break;

  case clk256:
    setBit(reg, bit00, false);
    setBit(reg, bit01, false);
    setBit(reg, bit02, true);
    return 256;
    break;

  case clk1024:
    setBit(reg, bit00, true);
    setBit(reg, bit01, false);
    setBit(reg, bit02, true);
    return 1024;
    break;

  case extClkSourceFallingEdge:
    setBit(reg, bit00, false);
    setBit(reg, bit01, true);
    setBit(reg, bit02, true);
    break;

  case extClkSourceRisingEdge:
    setBit(reg, bit00, true);
    setBit(reg, bit01, true);
    setBit(reg, bit02, true);
    break;

  default:
    setBit(reg, bit00, false);
    setBit(reg, bit01, false);
    setBit(reg, bit02, false);
    break;
  }

  return 0;
}








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






/**
 * @brief puts the ATmega328P microcontroller into the specified sleep mode.
 *        This method disables interrupts, sets the desired sleep mode, sets the power reduction bits,
 *        enables or disables BOD sleep mode based on the 'enableBODSleep' parameter, delays for three clock cycles,
 *        enables the sleep mode, enables interrupts, enters sleep mode, resumes execution after waking up,
 *        clears the power reduction bits, optionally enables BOD after sleep mode, and finally disables the sleep mode.
 * 
 * @param sleepMode The sleep mode to enter.
 * @param powerReductionBits The power reduction bits to set.
 * @param enableBODSleep Whether to enable BOD sleep mode or not.
 * 
 * 
 * @note Use Standby and Extended Standby sleep modes only with external crystals or resonators.
 * 
 * @warning This method assumes that the required functions 'setBit()', 'clearBit()', 'setBitMask()', 'clearBitMask()',
 *          'BITMASK()', 'PRR', and 'MCUCR' are properly defined.
 * 
 * @example 
 * MyAtmega328p avr;
 * avr.sleep(powerDown, 0x01, false);
 */
void MyAtmega328p::sleep(sleepModes sleepMode, uint8_t powerReductionBits, bool enableBODSleep) {

  // Disable interrupts
  cli();

  // Disable sleep mode
  setBit(MCUCR, SE, false);

  // Set the requested sleep mode
  switch(sleepMode) {
    case idle:
      clearBitMask(SMCR, BITMASK(SM0) | BITMASK(SM1) | BITMASK(SM2));
      break;
    case adcNoiseReduction:
      setBitMask(SMCR, BITMASK(SM0));
      clearBitMask(SMCR, BITMASK(SM1) | BITMASK(SM2));
      break;
    case powerDown:
      setBitMask(SMCR, BITMASK(SM1));
      clearBitMask(SMCR, BITMASK(SM2) | BITMASK(SM0)); 
      break;
    case powerSave:
      setBitMask(SMCR, BITMASK(SM0) | BITMASK(SM1));
      clearBitMask(SMCR, BITMASK(SM2));
      break;
    case Standby:
      // only recommend for use with external crystals or resonators
      clearBitMask(SMCR, BITMASK(SM0));
      setBitMask(SMCR, BITMASK(SM1) | BITMASK(SM2));
      break;
    case extendedStandby:
      // only recommend for use with external crystals or resonators
      setBitMask(SMCR, BITMASK(SM0) | BITMASK(SM1) | BITMASK(SM2) ); 
      break;
    default:
      // Invalid sleep mode, do nothing
      return;
  }

  // Set the power reduction bits
  PRR |= powerReductionBits;

  // Enable BODS bit and set BODS to disable BOD in sleep mode if enableBODSleep is true
  if (enableBODSleep) {
    setBit(MCUCR, BODS);
    setBit(MCUCR, BODSE);
    clearBit(MCUCR, BODSE);
  }

  // Delay for three clock cycles
  asm volatile("nop");
  asm volatile("nop");
  asm volatile("nop");

  // Enable sleep mode
  setBit(SMCR, SE);

  // Enable interrupts
  sei();

  // Enter sleep mode
  asm volatile("sleep");

  /*
  Execution will resume here after waking up
  ...
  */

  // Clear the power reduction bits
  PRR &= ~powerReductionBits;

  // Re-enable BOD after sleep mode if enableBODSleep is true (optional)
  if (enableBODSleep) {
    clearBit(MCUCR, BODS);
  }

  // Disable sleep mode
  setBit(MCUCR, SE, false);
  clearBitMask(SMCR, BITMASK(SM0) | BITMASK(SM1) | BITMASK(SM2));
}









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



