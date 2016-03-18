#include <Arduino.h>

#include <DMXSerial.h>
#include <SPI.h>

// Following defines is for the shift register

// #define SPI_sck   13
// #define SPI_miso  12
// #define SPI_mosi  11
// #define SPI_ss    10
//#define PLSR_SH_LD_pin  6

#define smokePin  4

int DMXaddr = 1;
int recValue = 0;
// Following definitions are for the 9th bit in address read protocol
// int MSB = 0;
// int MSBPin = 7;

int colourPin[3] = {5, 6, 9};
int greenPin = 6;
int bluePin = 9;
int redPin = 5;

int RGB[3] = {0, 0, 0};

//This function calculates address using the read data from shift register and MSB status

// int getAddress(byte setValue, int MSB) {
//   int address = (int) setValue;
//   if(MSB == LOW) address = 256 + address;
//   return address;
// }
int* readColour(int DMXaddr){
  static int colours[3];

  for (int i = 0; i<3; i++){
    colours[i] = DMXSerial.read((DMXaddr+i));
  }

  return colours;
}

void writeColour(int dimmer, int *colour){
  for (int i = 0; i < 3; i++){
    int c = colour[i] * dimmer;
    int mapC = map(c, 0, 65025, 0, 255);
    analogWrite(colourPin[i], mapC);
  }
}


void setup() {
  DMXSerial.init(DMXReceiver);          // initialize DMX as receiver
  // SPI.begin();                            // start SPI interface which is used for shift register
  // SPI.setDataMode(SPI_MODE0);             // setting mode for shift register
  // SPI.setBitOrder(MSBFIRST);              // says how the data will be ordered, MSB first
  // SPI.setClockDivider(SPI_CLOCK_DIV2);    //

  //pinMode(PLSR_SH_LD_pin, OUTPUT);        // set it as output
  //pinMode(smokePin, OUTPUT);              // set up pin 7 as output for smoke maschine
  pinMode(greenPin, OUTPUT);
  pinMode(redPin, OUTPUT);
  pinMode(bluePin, OUTPUT);


// Tell the SN74HC165 Parallel-load shift register to poll the inputs
  //digitalWrite(PLSR_SH_LD_pin, LOW);
  // put your setup code here, to run once:
  //pinMode(MSBPin, INPUT);


}

void loop() {
  // Latch the inputs into the shift register
  //digitalWrite(PLSR_SH_LD_pin, HIGH);
  //digitalWrite(7, LOW);

  // Read in all 8 inputs of the SN74HC165 into a byte
  //byte DMX_addr = SPI.transfer(0x00);

  // Read MSB state
  //MSB = digitalRead(MSBPin);
  // Calculate address
  //DMXaddr = getAddress(DMX_addr, MSB);

  // For testing the address is hard coded
  DMXaddr = 1;
  int *colour;
  //recValue = DMXSerial.read(DMXaddr);

  // For testing RGB LED is used with PWM outputs for each color
  int dimmer = DMXSerial.read(DMXaddr);
  colour = readColour((DMXaddr+1));
  writeColour(dimmer, colour);
  // analogWrite(redPin, DMXSerial.read(DMXaddr));
  // analogWrite(greenPin, DMXSerial.read((DMXaddr+1)));
  // analogWrite(bluePin, DMXSerial.read((DMXaddr+2)));


  // If thers is no DMX signal for 5 seconds then light up red LED
  unsigned long lasPacket = DMXSerial.noDataSince();
  // if(recValue > 150) digitalWrite(smokePin, HIGH);
  // else digitalWrite(smokePin, LOW);
  if (lasPacket > 5000) {
    analogWrite(A0, 200*4);
  } else analogWrite(A0, 0);
  //digitalWrite(PLSR_SH_LD_pin, LOW);
}
