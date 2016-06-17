#include <Arduino.h>
#include <DMXSerial.h>           // when using DMX uncomment this
#include <SPI.h>

/**
   Arduino Connections for Shift Register:

   Connect [D13] to the Clock pin of the SN74HC165 (pin 2)
   Connect [D12] to the Qh pin of the SN74HC165 (pin 9)
   Connect [D11] to nothing.  It's not used in this sketch
   Connect [D9]  to an LED for testing
   Connect [D6]  to the SH/LD pin of the SN74HC165 (pin 1)

 **/

// Following defines is pin connections

#define SPI_sck   13
#define SPI_miso  12
#define SPI_mosi  11
#define SPI_ss    10
#define PLSR_SH_LD_pin  6
#define smokePin  4
// #define greenPin  2
// #define bluePin   9
// #define redPin    5
#define MSBPin    7
#define setPin    8


// Initialize variable
int DMXaddr = 1;
int recValue = 0;
// Following definitions are for the 9th bit in address read protocol
int MSB = 0;

// int colourPin[3] = {5, 6, 9};

// Array for storing colours for test RGB LED
// int RGB[3] = {0, 0, 0};

// Initialize variables for address is set
int stateAddr = 0;
int previousAddr = 0;

//This function calculates address using the read data from shift register and MSB status

int getAddress(byte setValue, int MSB) {
        int address = (int) setValue;
        if(MSB == HIGH) return (256 + address);
        // return address;
}

// readColour function reads address from the DMX input
// argument DMXaddr is the user set address

// int* readColour(int DMXaddr){
//   static int colours[3];
//
//   for (int i = 0; i<3; i++){
//     colours[i] = DMXSerial.read((DMXaddr+i));
//   }
//   return colours;
// }

// writeColour function outputs PWM data for test LED
// arguments - dimmer - value of read dimmer data, used to modify colour data
// arguments - colour - pointer to colour array

// void writeColour(int dimmer, int *colour){
//   //int strobe = DMXSerial.read(DMXaddr + 5);
//
//   for (int i = 0; i < 3; i++){
//     int c = colour[i] * dimmer;
//     int mapC = map(c, 0, 65025, 0, 255);
//       analogWrite(colourPin[i], mapC);
//   }
// }


void setup() {
        DMXSerial.init(DMXReceiver);    // initialize DMX as receiver
        // Serial.begin(115200);                   // used for address seting test. Comment out when DMX is being used

        SPI.begin();                      // start SPI interface which is used for shift register
        SPI.setDataMode(SPI_MODE0);       // setting mode for shift register
        SPI.setBitOrder(MSBFIRST);        // says how the data will be ordered, MSB first
        SPI.setClockDivider(SPI_CLOCK_DIV2); //

        pinMode(PLSR_SH_LD_pin, OUTPUT);  // set pins as outputs
        pinMode(smokePin, OUTPUT);        // set up pin 7 as output for smoke maschine
        // pinMode(greenPin, OUTPUT);
        // pinMode(redPin, OUTPUT);
        // pinMode(bluePin, OUTPUT);

        pinMode(setPin, INPUT);           // set pins as inputs
        pinMode(MSBPin, INPUT);

        // Tell the SN74HC165 Parallel-load shift register to poll the inputs
        digitalWrite(PLSR_SH_LD_pin, LOW);
        // put your setup code here, to run once:
        // Serial.println("Initialized!");
}

void loop() {
        // Latch the inputs into the shift register
        digitalWrite(PLSR_SH_LD_pin, HIGH);

        // Read in all 8 inputs of the SN74HC165 into a byte
        byte DMX_addr = SPI.transfer(0x00);

        // read set state pin
        stateAddr = digitalRead(setPin);
        // Serial.println(stateAddr);
        //if set state pin has changed from previous time then set address
        if(stateAddr != previousAddr) {
                // Read MSB state
                MSB = digitalRead(MSBPin);
                // Calculate address
                DMXaddr = getAddress(DMX_addr, MSB);
                // set previous state to read state
                previousAddr = stateAddr;
                // Serial.print("Address ");
                // Serial.print(DMXaddr);
                // Serial.println("is set");
        }

        // int *colour;
        recValue = DMXSerial.read(DMXaddr);
        analogWrite(smokePin, recValue);

        // For testing RGB LED is used with PWM outputs for each color
        // int dimmer = DMXSerial.read(DMXaddr);
        // colour = readColour((DMXaddr+1));
        // writeColour(dimmer, colour);

        // when not using writeColour function, use these
        // analogWrite(redPin, DMXSerial.read(DMXaddr));
        // analogWrite(greenPin, DMXSerial.read((DMXaddr+1)));
        // analogWrite(bluePin, DMXSerial.read((DMXaddr+2)));


        // If thers is no DMX signal for 5 seconds then light up red LED
        unsigned long lasPacket = DMXSerial.noDataSince();
        if(recValue > 150) digitalWrite(smokePin, HIGH);
        else digitalWrite(smokePin, LOW);
        if (lasPacket > 5000) {
                analogWrite(A0, 200*4);
        } else analogWrite(A0, 0);

        digitalWrite(PLSR_SH_LD_pin, LOW);
}
