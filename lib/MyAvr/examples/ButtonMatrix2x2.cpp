/*
Das hier ist ein Beispiel für eine Anwendung mit 2x2 button matrix.
Es wird während des codes ein pin auf input mit pullup widerstand konfiguriert
Ein pin wird als output sink konfiguriert um den Ground zu bilden.
die zwei weiteren pins werden auf output high gesetzt um störungen zu vermeiden.

Im Beispiel wurde die Matrix folgendermaßen angeschlossen:
L1 -> PD2
L2 -> PD3
R1 -> PD4
R2 -> PD5


*/



#include <MyAvr.hpp>



// Objekt für LED Streifen im Wohnzimmer
MyAtmega328p myAVR(8000000UL);

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