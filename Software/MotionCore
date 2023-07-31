#include <SPI.h>
#include <digitalWriteFast.h>


void setup() {
  SPI.setBitOrder(MSBFIRST);//Most Significant Bit First
  SPI.setDataMode(SPI_MODE0);// Mode 0 Rising edge of data, keep clock low
  SPI.setClockDivider(SPI_CLOCK_DIV2);//Run the data in at 16MHz/2 - 8MHz
  SPI.begin();
  Serial.begin(9600);

  //Timer Setup ********
  TCCR1A = B00000000;//Register A all 0's since we're not toggling any pins
  TCCR1B = B00001011;//bit 3 set to place in CTC mode, will call an interrupt on a counter match
  //bits 0 and 1 are set to divide the clock by 64, so 16MHz/64=250kHz
  TIMSK1 = B00000010;//bit 1 set to call the interrupt on an OCR1A match
  OCR1A = 30;
  interrupts();               //Enable interrupt
  //TIMER SETUP END


}

void loop () {
  int mode = 1;
}

ISR(TIMER1_COMPA_vect) {
  SPI.transfer(0x01);
}
