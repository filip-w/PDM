#include <SPI.h>
#include <mcp2515.h>
#include <FIR.h>
#include <ACS712.h>

//General setup
#define BaseCanID 0x500
#define UpdateRate 10 //Frontend thermocouple scan rate and CAN-message update rate in Hz.
#define serialDebug false
#define defaultOutputState HIGH

struct OutputChannel{
   int SwitchOutputChannel;
   float Ratedcurrent;
   float timeToBlow;
   ACS712  ACS;
};

struct can_frame canMsg1;
enum CAN_SPEED BaudRate;

volatile bool interrupt = false;
struct can_frame frame;

//CAN interrupt
void irqHandler() {
    interrupt = true;
}

//Digital Pins
//Digital outputs
#define IN_CHN1        3
#define IN_CHN2        6
#define IN_CHN3        5
#define IN_CHN4        4
#define CAN_ChipSelect 10

//Digital Inputs
#define DI1            7
#define DI2            8
#define BT_STATE       9
#define CAN_Interrupt  2 

// These constants won't change. They're used to give names to the pins used:
const int analogInPin5 = A4;  // Battery reference
const float barRestistorDiv = 1/(1+3.9);
const int analogInPin6 = A5;  // Analog input 1 (Lower pot on simulator)
const int analogInPin7 = A6;  // Analog input 2 (high pot on simulator)
const int analogInPin8 = A7;  // Analog input 3 (NC on simulator)

int sensorValue = 0;  // value read from the pot
int outputValue = 0;  // value output to the PWM (analog out)

MCP2515 mcp2515(CAN_ChipSelect);

void setup() {
  OutputChannel CHList[4] = {
  {3, 5, 1, (A0, 5.0, 1023, 100)},
  {6, 5, 1, (A1, 5.0, 1023, 100)}, 
  {5, 5, 1, (A2, 5.0, 1023, 100)}, 
  {4, 5, 1, (A3, 5.0, 1023, 100)},
  };

  pinMode(DI1, INPUT_PULLUP);
  pinMode(DI2, INPUT_PULLUP);

  Serial.begin(9600);

  while (!Serial) delay(1); // wait for Serial on Leonardo/Zero, etc

  Serial.println("Initializing sensors...");
  
  delay(500); // wait for MAX chip to stabilize

  int canid = BaseCanID;
  Serial.print("CANid: ");
  Serial.println(canid);

  //Set baudrate
  BaudRate = CAN_500KBPS;
  Serial.println("500KBPS");

  canMsg1.can_id  = canid;
  canMsg1.can_dlc = 8;
  canMsg1.data[0] = 0x00;
  canMsg1.data[1] = 0x00;
  canMsg1.data[2] = 0x00;
  canMsg1.data[3] = 0x00;
  canMsg1.data[4] = 0x00;
  canMsg1.data[5] = 0x00;
  canMsg1.data[6] = 0x00;
  canMsg1.data[7] = 0x00;

  Serial.println("Initializing CAN module...");
  mcp2515.reset();
  mcp2515.setBitrate(BaudRate,MCP_8MHZ);
  mcp2515.setNormalMode();
  Serial.println("DONE.");

  for (int i = 0; i <= 4; i++) {
    pinMode(CHList[i].SwitchOutputChannel, OUTPUT);   //Setup digital output pins
    digitalWrite(CHList[i].SwitchOutputChannel, defaultOutputState);  // Set Default state
    //CHList[i].ACS.autoMidPoint(); // Calibrate current sensors
  } 

  pinMode(CAN_Interrupt, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(CAN_Interrupt), irqHandler, FALLING);
}

void loop() {
  // int mA = ACS.mA_DC();
  // read the analog in value:
  // sensorValue = analogRead(analogInPin5);
  // map it to the range of the analog out:
  // outputValue = map(sensorValue, 0, 1023, 0, 245); //Scaling for Battery reference
  // change the analog out value:
  int reading1 = digitalRead(DI1);
  int reading2 = digitalRead(DI2);
  //int reading3 = digitalRead(CAN_Interrupt);

  // print the results to the Serial Monitor:
  Serial.print("Dig1 = ");
  Serial.print(reading1);
  Serial.print(" Dig2 = ");
  Serial.print(reading2);
  Serial.print(" Current = ");
  //Serial.print(mA);
  Serial.print("\t Battery reference = ");
  Serial.println(outputValue);


  canMsg1.data[0] = reading1;


  //mcp2515.sendMessage(&canMsg1);
  //mcp2515.sendMessage(&canMsg2);
  delay(float(1)/UpdateRate*1000); //Crude wait, should use a dynamic time offset depending.

  if (interrupt) {
    Serial.println("Rx msg!");
        interrupt = false;

        uint8_t irq = mcp2515.getInterrupts();

        if (irq & MCP2515::CANINTF_RX0IF) {
            if (mcp2515.readMessage(MCP2515::RXB0, &frame) == MCP2515::ERROR_OK) {
                // frame contains received from RXB0 message
            }
        }

        if (irq & MCP2515::CANINTF_RX1IF) {
            if (mcp2515.readMessage(MCP2515::RXB1, &frame) == MCP2515::ERROR_OK) {
                // frame contains received from RXB1 message
            }
        }
    }
}