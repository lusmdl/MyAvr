#include <MyAvr.hpp>



// Objekt für LED Streifen im Wohnzimmer
MyAtmega328p myAVR(F_CPU);

int main(void)
{
  
  // teste den Mikrocontroller
  myAVR.test(100);

  // initialisiere die UART
  myAVR.initUart(9600);

  // konfiguriere Taster
  uint8_t matrixPins[4] {PD2, PD3, PD4, PD5};
  MyButtonMatrix2x2 btnMatrix(DDRD, PORTD, PIND, matrixPins);


  while (1)
  {
  /* code */
  
  myAVR.printUart("\nBEGIN\n");

  for ( int i = 0; i < 4; i++)
  {
    /* code */
    myAVR.printUart ("button #" + String(i) + ": " +  String(btnMatrix.getButtonStatus(i)) + "\n");
  
  }
    myAVR.delayMs(1000);
  
  
  
  }
  return 0;
}