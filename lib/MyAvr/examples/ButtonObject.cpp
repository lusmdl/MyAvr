/**
 * Funktion zum Testen eines Mikrocontrollers
 * und zur Konfiguration eines Tasters
 * 
 * @param None
 * 
 * @return None
 */

#include <MyAvr.hpp>

MyAtmega328p myAVR(8000000UL);

int main (void) {

  // teste den Mikrocontroller
  myAVR.test(100);

  // initialisiere die UART
  myAVR.initUart(9600);

  // konfiguriere die GPIOs
  myAVR.setGpioConfig(inputPullUp, DDRD, PORTD, PD3);

  MyButton btnTest(PIND, PD3, true);
  
  while (1) {

    if (btnTest.getStatus())
    {
      /* Wenn der Taster gedrückt wird */
      myAVR.printUart("btnTest pressed!\n");
      myAVR.delayMs(1000);
    }

  }
  return 0;
}