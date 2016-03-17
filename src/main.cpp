#include <Arduino.h>

#include <DMXSerial.h>
#include <SPI.h>

#define SPI_sck   13
#define SPI_miso  12
#define SPI_mosi  11
#define SPI_ss    10

#define PLSR_SH_LD_pin  6

#define smokePin  4

int DMXaddr = 1;
int recValue = 0;
int MSB = 0;
int MSBPin = 7;
int greenPin = 4;
int bluePin = 9;
int redPin = 5;

int getAddress(byte setValue, int MSB) {
  int address = (int) setValue;
  if(MSB == LOW) address = 256 + address;
  return address;
}


void setup() {
  DMXSerial.init(DMXReceiver);          // initialize DMX as receiver
  SPI.begin();                            // start SPI interface which is used for shift register
  SPI.setDataMode(SPI_MODE0);             // setting mode for shift register
  SPI.setBitOrder(MSBFIRST);              // says how the data will be ordered, MSB first
  SPI.setClockDivider(SPI_CLOCK_DIV2);    // hujzin :D

  pinMode(PLSR_SH_LD_pin, OUTPUT);        // set it as output
  pinMode(smokePin, OUTPUT);              // set up pin 7 as output for smoke maschine
  pinMode(greenPin, OUTPUT);
  pinMode(redPin, OUTPUT);
  pinMode(bluePin, OUTPUT);


// Tell the SN74HC165 Parallel-load shift register to poll the inputs
  digitalWrite(PLSR_SH_LD_pin, LOW);
  // put your setup code here, to run once:
  pinMode(MSBPin, INPUT);


}

void loop() {
  // Latch the inputs into the shift register
  digitalWrite(PLSR_SH_LD_pin, HIGH);
  digitalWrite(7, LOW);

  // Read in all 8 inputs of the SN74HC165 into a byte
  //byte DMX_addr = SPI.transfer(0x00);
  //MSB = digitalRead(MSBPin);
  //DMXaddr = getAddress(DMX_addr, MSB);
  DMXaddr = 1;
  recValue = DMXSerial.read(DMXaddr);
  analogWrite(redPin, DMXSerial.read(DMXaddr));
  analogWrite(greenPin, DMXSerial.read(DMXaddr+1));
  analogWrite(bluePin, DMXSerial.read(DMXaddr+2));
  unsigned long lasPacket = DMXSerial.noDataSince();
  if(recValue > 150) digitalWrite(smokePin, HIGH);
  else digitalWrite(smokePin, LOW);
  if (lasPacket > 5000) {
   analogWrite(A0, 200*4);
  } else analogWrite(A0, 0);


  digitalWrite(PLSR_SH_LD_pin, LOW);
}
