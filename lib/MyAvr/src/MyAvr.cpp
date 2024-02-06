
#include "MyAvr.hpp"





/* 
Konstruktor der Klasse MyAtmega328p
Initialisiert die CPU-Frequenz des Mikrocontrollers.
Parameter:
- freq: Die gewünschte CPU-Frequenz des Mikrocontrollers 
*/
MyAtmega328p ::MyAtmega328p(unsigned long freq)
{
  // konfiguriere die CPU frequenz
  cpuFreq = freq;

}








/*
   Schreibt eine Zeichenkette in das EEPROM.
   Parameter:
   - address: Adresse im EEPROM, in die die Zeichenkette geschrieben werden soll
   - str: Die zu schreibende Zeichenkette
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












/*
   Liest einen String aus dem EEPROM an einer gegebenen Adresse in einen Puffer.
   Parameter:
   - address: Adresse im EEPROM, von der der String gelesen werden soll
   - str: Referenz auf einen String, in den der gelesene String gespeichert werden soll
   - bufferSize: Größe des Puffers für den gelesenen String
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















/*
   Schreibt ein Byte in das EEPROM.
   Parameter:
   - address: Adresse im EEPROM, in die das Byte geschrieben werden soll
   - value: Das zu schreibende Byte 
*/
void MyAtmega328p ::writeToEEPROM(uint16_t address, uint8_t value) {
  // Speicher-Zugriff einschränken auf Adressebereich des EEPROM
  if (address >= 0 && address < E2END) {
    eeprom_write_byte((uint8_t*)address, value);
  }
}
  




/*
   Liest ein Byte aus dem EEPROM.
   Parameter:
   - address: Adresse im EEPROM, von der das Byte gelesen werden soll

   Rückgabewert:
   - Das gelesene Byte 
*/
uint8_t MyAtmega328p ::readFromEEPROM(uint16_t address) {
  // Überprüfen, ob Adresse innerhalb des EEPROM-Bereichs liegt
  if (address >= 0 && address < E2END) {
    return eeprom_read_byte((uint8_t*)address);
  }
  
  return 0; // Rückgabewert, wenn Adresse ungültig ist
}











/*
   Gibt den Wert eines bestimmten Bits in einem Register zurück.
   Parameter:
   - reg: Das Register, aus dem das Bit gelesen werden soll
   - bit: Die Position des Bits im Register
   Rückgabewert:
   - Der Wert des Bits (true, wenn das Bit gesetzt ist, false sonst) 
*/
bool MyController::getBit(volatile uint8_t &reg, uint8_t bit)
{
  return ((reg & (1 << bit)) != 0);


}













/*
   Setzt ein bestimmtes Bit in einem Register auf einen Wert.
   Parameter:
   - reg: Das Register, in dem das Bit gesetzt werden soll
   - bit: Die Position des Bits im Register
   - value: Der Wert, auf den das Bit gesetzt werden soll (true für 1, false für 0)
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
 * Löscht das angegebene Bit in einem Register.
 *
 * @param reg Das Register, in dem das Bit gelöscht werden soll.
 * @param bit Das zu löschende Bit.
 *
 * @remarks Diese Methode setzt das angegebene Bit auf 0, indem es eine Bitmaske erstellt, die das Bit invertiert, und dann eine Bit-UND-Operation mit dem Register durchführt.
 *
 * @note    Diese Methode geht davon aus, dass das Register als volatil deklariert ist und das Bit im richtigen Format übergeben wird.
 *
 * @warning Diese Methode kann zu unvorhersehbarem Verhalten führen, wenn das Bit falsch oder das Register nicht korrekt definiert ist.
 */
void MyController::clearBit(volatile uint8_t &reg, uint8_t bit) {
  
  reg &= ~(1 << bit); // set the bit to 0
  
}






/**
 * Setzt oder löscht die Bitmaske in einem angegebenen Register basierend auf dem angegebenen Wert.
 *
 * @param reg Das Register, in dem die Bitmaske gesetzt oder gelöscht werden soll.
 * @param bitMask Die zu setzende oder zu löschende Bitmaske.
 * @param value Der Wert, der angibt, ob die Bitmaske gesetzt oder gelöscht werden soll.
 *
 * @remarks Diese Methode setzt das Bit auf 1 oder 0 im angegebenen Register, abhängig vom Wert-Parameter.
 *
 * @note    Diese Methode geht davon aus, dass das Register als volatil deklariert ist und die Bitmaske im richtigen Format übergeben wird.
 *
 * @warning Diese Methode kann zu unvorhersehbarem Verhalten führen, wenn die Bitmaske falsch oder das Register nicht korrekt definiert ist.
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
 * Löscht die Bitmaske in einem angegebenen Register.
 *
 * @param reg Das Register, in dem die Bitmaske gelöscht werden soll.
 * @param bitMask Die zu löschende Bitmaske.
 *
 * @remarks Diese Methode setzt das Bit auf 0 im angegebenen Register, indem es die Bitmaske invertiert und eine Bit-UND-Operation durchführt.
 *
 * @note    Diese Methode geht davon aus, dass das Register als volatil deklariert ist und die Bitmaske im richtigen Format übergeben wird.
 *
 * @warning Diese Methode kann zu unvorhersehbarem Verhalten führen, wenn die Bitmaske falsch oder das Register nicht korrekt definiert ist.
 */
void MyController::clearBitMask(volatile uint8_t &reg, uint8_t bitMask) {

  reg &= ~bitMask; // set the bit to 0
  
}









/*
   Setzt die GPIO-Konfiguration abhängig vom gewählten Modus.
   Parameter:
   - mode: Der gewünschte GPIO-Modus
   - DDxn: Das entsprechende Datenrichtungsregister
   - PORTxn: Das entsprechende Portregister
   - bit: Die Position des Bits im Register
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












/*
   Verzögert die Ausführung für die angegebene Anzahl von Millisekunden.
   Parameter:
   - ms: Anzahl der zu verzögernden Millisekunden 
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
 * @brief Delay the execution for specified number of microseconds
 * @param microseconds Number of microseconds to delay the execution for
 * @param cpuFrequency The CPU frequency in Hz
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










/*
   Testet die Funktion der Mikrocontroller-LED.
   Schaltet die LED für eine bestimmte Zeit ein und dann aus.
   Parameter:
   - delay: Die Wartezeit in Millisekunden zwischen dem Ein- und Ausschalten der LED
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










/*
   Initialisiert die UART-Kommunikation auf dem MyAtmega328p.
   Konfiguriert die Baudrate, die Frame-Formateinstellungen und
   gibt eine "Hello World!"-Nachricht über UART aus.
   Parameter:
   - baudrate: Die gewünschte Baudrate für die UART-Kommunikation
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










/*
   Schreibt ein Zeichen in den UART-Ausgabepuffer.
   Das Zeichen wird über die Hardware-Schnittstelle des Mikrocontrollers (UART) gesendet.
   Parameter:
   - c: Das zu sendende Zeichen
   Hinweise:
   - Die Funktion blockiert, bis der Ausgabepuffer bereit ist, ein Zeichen zu akzeptieren.
*/
void MyAtmega328p ::putcharUart(char c)
{
  while (!(UCSR0A & (1 << UDRE0)))
    ;
  UDR0 = c;
}















/*
   Gibt einen Zeichenkettenparameter über das Uart aus.
   Parameter:
   - str: Die zu übertragende Zeichenkette 
*/
void MyAtmega328p ::printUart(const char *str)
{
  const size_t len = strlen(str);
  for (size_t i = 0; i < len; i++)
  {
    putcharUart(str[i]);
  }
}













/*
   Druckt einen String über die UART-Schnittstelle.
   Parameter:
   - str: Der zu druckende String
*/
void MyAtmega328p::printUart(const String &str)
{
  const size_t len = str.length();
  for (size_t i = 0; i < len; i++)
  {
    putcharUart(str.charAt(i));
  }
}













/*
   Führt einen NOP (No Operation) Befehl aus.
   Dieser Befehl wird verwendet, um eine Leerlauf-Funktion auszuführen und
   kann zur Synchronisation oder zur Verzögerung verwendet werden.
*/
void MyController ::nop()
{
  asm volatile("nop");
}



/*
Setzt die Konfiguration für den Timer/Counter 0 (TC0) auf dem Atmega328p.

Parameter:
- mode: Der gewünschte Modus für den TC0 (z.B. ctc - Clear Timer on Compare Match)
- prescaler: Der gewünschte Vorteiler für den TC0
- topTime: Der gewünschte Vergleichswert für den CTC-Modus


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
void MyAtmega328p ::setTC0Config(tcModes mode, tcPrescalers prescaler, time topTime)
{


  switch (mode)
  {
  case ctc:

    // prescaler
    auto N = setTC01Prescaler(prescaler, TCCR0B, CS02, CS01, CS00);

    // ctc modus einrichten
    setBit(TCCR0A, WGM00, 0);
    setBit(TCCR0A, WGM01, 1);
    setBit(TCCR0B, WGM02, 0);

    // Setze den Vergleichswert für den CTC-Modus
    auto clkPerMillisekond { ( cpuFreq / N ) / 1000 };
    OCR0A = ( ( clkPerMillisekond * topTime ) - 1 );
       

    // interrupt aktivieren
    setBit(TIMSK0, OCIE0A);
    sei();

    

    break;

  default:
    break;
  }

}

/*
Setzt den Prescaler für den Timer/Counter0 auf einen bestimmten Wert.
   Parameter:
   - prescaler: Der gewünschte Prescaler-Wert
   - reg: Das Register, in dem die Bits für den Prescaler gesetzt werden sollen
   - bit02: Die Position des Bit02 im Register
   - bit01: Die Position des Bit01 im Register
   - bit00: Die Position des Bit00 im Register
   Rückgabewert:
   - Der gesetzte Prescaler-Wert (1, 8, 64, 256, 1024 oder 0 für externe Taktquelle)

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
uint32_t MyAtmega328p ::setTC01Prescaler(tcPrescalers prescaler, volatile uint8_t &reg, uint8_t bit02, uint8_t bit01, uint8_t bit00)
{

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








/* 
   Konstruktor der Klasse MyButton
   Initialisiert die Member-Variablen der Klasse.
   Parameter:
   - PINXn: Eine Referenz auf das Register, das den Status des Buttons enthält.
   - bitPosition: Die Position des Bits im Register, das den Button-Status repräsentiert.
*/
MyButton ::MyButton(volatile uint8_t& PINXn, uint8_t bitPosition, bool invertButton) : registerPtr(&PINXn), bit(bitPosition), pushed(false), numbersGetPushed(0), paraInvert(invertButton) {}












/*
   Gibt den Status des Buttons zurück.
   Überprüft den Zustand des Buttons und aktualisiert die Anzahl der gedrückten Tasten.
   Rückgabewert:
   - Der Status des Buttons (true, wenn der Button gedrückt ist, false sonst)
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


/*
   Setzt den Status des Buttons.
   Parameter:
   - newValue: Der neue Wert für die Anzahl der gedrückten Buttonnummern (Standardwert ist 0)
*/
void MyButton ::setStatus(uint32_t newValue) {
  numbersGetPushed = newValue;
  if (newValue == 0) {
    pushed = false;
  }
}






/**
 * Versetzt den ATmega328P-Mikrocontroller in den angegebenen Schlafmodus.
 *
 * @param sleepMode Der einzugehende Schlafmodus.
 * @param powerReductionBits Die zu setzenden Power-Reduktionsbits.
 * @param enableBODSleep Ob der BOD-Schlafmodus aktiviert werden soll oder nicht.
 *
 * @remarks Diese Methode deaktiviert Interrupts, setzt den gewünschten Schlafmodus, setzt die Power-Reduktionsbits,
 *          aktiviert oder deaktiviert den BOD-Schlafmodus basierend auf dem 'enableBODSleep'-Parameter, verzögert für drei Taktzyklen,
 *          aktiviert den Schlafmodus, aktiviert Interrupts, geht in den Schlafmodus, nimmt die Ausführung nach dem Aufwachen wieder auf,
 *          löscht die Power-Reduktionsbits, aktiviert optional den BOD nach dem Schlafmodus und deaktiviert schließlich den Schlafmodus.
 *
 * @note    Verwende Schlafmodi Standby und Extended Standby nur mit externen Kristallen oder Resonatoren.
 *
 * @warning Diese Methode geht davon aus, dass die erforderlichen Funktionen 'setBit()', 'clearBit()', 'setBitMask()', 'clearBitMask()',
 *          'BITMASK()', 'PRR' und 'MCUCR' ordnungsgemäß definiert sind.
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









/* 
   Konstruktor der Klasse MyButtonMatrix2x2
   Initialisiert die Member-Variablen der Klasse.
*/
MyButtonMatrix2x2 ::MyButtonMatrix2x2(volatile uint8_t& DDXn, volatile uint8_t& PORTXn, volatile uint8_t& PINXn, uint8_t bitPosition[4] ) : registerPtrDataDirection(&DDXn), registerPtrOutput(&PORTXn), registerPtrInput(&PINXn)  {

  for (uint8_t i = 0; i < 4; i++)
  {
    bit[i] = bitPosition[i];
  }
  

  



}







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

  
  return getBit(*registerPtrInput, bit[sense])? true : false;

}



