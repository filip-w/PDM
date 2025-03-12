//trouble shoot channel 2, does not turn on?

#include <SPI.h>
#include <mcp2515.h>
#include <FIR.h>
#include <ACS712.h>

//General setup
#define BaseCanID 0x500
#define UpdateRate 10  //CAN-message update rate in Hz.
#define serialDebug false
#define defaultOutputState LOW
#define maxSystemCurrent 10000 //mA

struct OutputChannel {
  int SwitchOutputChannel;
  float Ratedcurrent;
  float timeToBlow;
  ACS712 *ACS;
  int canControlSignalOffset;
  int canFuseTrippedOffset;
  bool fusedTripped;
  int actualCurrent;
};

struct can_frame canMsg1;
struct can_frame canMsg2;
struct can_frame canMsg3;
enum CAN_SPEED BaudRate;

volatile bool interrupt = false;
struct can_frame ctrlMsg;
bool ctrlMsgRecieved = false;
int ResetFuseCh4  = 0;
int ResetFuseCh3  = 0;
int ResetFuseCh2  = 0;
int ResetFuseCh1  = 0;
int ControlCh1    = 0;
int ControlCh2    = 0;
int ControlCh3    = 0;
int ControlCh4    = 0;

int totalSystemActCurrent = 0;

//CAN interrupt
void irqHandler() {
  interrupt = true;
}

//Digital Pins
//Digital outputs
#define IN_CHN1 3
#define IN_CHN2 6
#define IN_CHN3 5
#define IN_CHN4 4
#define CAN_ChipSelect 10

//Digital Inputs
#define DI1 7
#define DI2 8
#define BT_STATE 9
#define CAN_Interrupt 2

// These constants won't change. They're used to give names to the pins used:
const int analogInPin5 = A4;  // Battery reference
const float barRestistorDiv = 1 / (1 + 3.9);
const int analogInPin6 = A5;  // Analog input 1 (Lower pot on simulator)
const int analogInPin7 = A6;  // Analog input 2 (high pot on simulator)
const int analogInPin8 = A7;  // Analog input 3 (NC on simulator)

int sensorValue = 0;  // value read from the pot
int outputValue = 0;  // value output to the PWM (analog out)

int sensorValue2 = 0;  // value read from the pot
int outputValue2 = 0;  // value output to the PWM (analog out)

int sensorValue3 = 0;  // value read from the pot
int sensorValue4 = 0;  // value read from the pot

MCP2515 mcp2515(CAN_ChipSelect);

ACS712 ACS0(A0, 5.0, 1023, 100);
ACS712 ACS1(A1, 5.0, 1023, 100);
ACS712 ACS2(A2, 5.0, 1023, 100);
ACS712 ACS3(A3, 5.0, 1023, 100);

OutputChannel CHList[4] = {
  { 3, 6000, 1, &ACS0, 7, 3 ,0 },
  { 6, 6000, 1, &ACS1, 6, 2 ,0 },
  { 5, 6000, 1, &ACS2, 5, 1 ,0 },
  { 4, 6000, 1, &ACS3, 4, 0 ,0 },
};



void setup() {
  pinMode(DI1, INPUT_PULLUP);
  pinMode(DI2, INPUT_PULLUP);

  Serial.begin(9600);

  while (!Serial) delay(1);  // wait for Serial on Leonardo/Zero, etc

  Serial.println("Initializing sensors...");

  delay(500);  // wait for MAX chip to stabilize

  int canid = BaseCanID;
  Serial.print("CANid: ");
  Serial.println(canid);

  //Set baudrate
  BaudRate = CAN_500KBPS;
  Serial.println("500KBPS");

  canMsg1.can_id = canid;
  canMsg1.can_dlc = 8;
  canMsg1.data[0] = 0x00;
  canMsg1.data[1] = 0x00;
  canMsg1.data[2] = 0x00;
  canMsg1.data[3] = 0x00;
  canMsg1.data[4] = 0x00;
  canMsg1.data[5] = 0x00;
  canMsg1.data[6] = 0x00;
  canMsg1.data[7] = 0x00;

  canMsg2.can_id = canid + 1;
  canMsg2.can_dlc = 8;

  canMsg3.can_id = canid + 2;
  canMsg3.can_dlc = 8;

  Serial.println("Initializing CAN module...");
  mcp2515.reset();
  mcp2515.setBitrate(BaudRate, MCP_8MHZ);

  //Filter out CAN configuration message
  mcp2515.setFilterMask(MCP2515::MASK0, false, 0x1FFFFFFF);
  mcp2515.setFilter(MCP2515::RXF0, false, BaseCanID+3);

  mcp2515.setNormalMode();
  Serial.println("DONE.");

  for (int i = 0; i <= 4; i++) {
    pinMode(CHList[i].SwitchOutputChannel, OUTPUT);                   //Setup digital output pins
    digitalWrite(CHList[i].SwitchOutputChannel, defaultOutputState);  // Set Default state
    //CHList[i].ACS.autoMidPoint(); // Calibrate current sensors
  }

  ACS0.autoMidPoint();  // Calibrate current sensors
  ACS1.autoMidPoint();  // Calibrate current sensors
  ACS2.autoMidPoint();  // Calibrate current sensors
  ACS3.autoMidPoint();  // Calibrate current sensors

  pinMode(CAN_Interrupt, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(CAN_Interrupt), irqHandler, FALLING);
}

void loop() {

  //Read current from each channel
  for (int i = 0; i <= 4; i++) {
    CHList[i].actualCurrent = CHList[i].ACS->mA_DC();
  }

  //Is current per chennel over the rated current? if so tripp fuse
  for (int i = 0; i <= 4; i++) {
    if (CHList[i].actualCurrent > CHList[i].Ratedcurrent){
      CHList[i].fusedTripped = true;
      digitalWrite(CHList[i].SwitchOutputChannel, false);
    }
  }

  //total current over system limit? tripp all fuses
  totalSystemActCurrent = CHList[0].actualCurrent + CHList[1].actualCurrent + CHList[2].actualCurrent + CHList[3].actualCurrent;

  if (totalSystemActCurrent > maxSystemCurrent) { 
    for (int i = 0; i <= 4; i++) {
      CHList[i].fusedTripped = true;
      digitalWrite(CHList[i].SwitchOutputChannel, false);
    }
  }

  canMsg2.data[0] = (CHList[0].actualCurrent >> 8);
  canMsg2.data[1] = (CHList[0].actualCurrent & 0xFF);
  canMsg2.data[2] = (CHList[1].actualCurrent >> 8);
  canMsg2.data[3] = (CHList[1].actualCurrent & 0xFF);
  canMsg2.data[4] = (CHList[2].actualCurrent >> 8);
  canMsg2.data[5] = (CHList[2].actualCurrent & 0xFF);
  canMsg2.data[6] = (CHList[3].actualCurrent >> 8);
  canMsg2.data[7] = (CHList[3].actualCurrent & 0xFF);

  // read the analog in value:
  sensorValue = analogRead(analogInPin5);
  int test = analogRead(A0);
  // map it to the range of the analog out:
  outputValue = map(sensorValue, 0, 1023, 0, 245);  //Scaling for Battery reference
  // change the analog out value:
  int reading1 = digitalRead(DI1);
  int reading2 = digitalRead(DI2);
  //int reading3 = digitalRead(CAN_Interrupt);


    // read the analog in value:
  sensorValue2 = analogRead(analogInPin6);
  sensorValue3 = analogRead(analogInPin7);
  sensorValue4 = analogRead(analogInPin8);
  // map it to the range of the analog out:
  outputValue2 = map(sensorValue2, 0, 1023, 0, 100);  //Scaling for 0-100 %

  //Prepare CAN message
  
  //message 1 "input"
  //Digital input
  canMsg1.data[0] = reading1;
  //Analog input
  canMsg1.data[1] = highByte(sensorValue2);
  canMsg1.data[2] = lowByte(sensorValue2);
  canMsg1.data[3] = highByte(sensorValue3);
  canMsg1.data[4] = lowByte(sensorValue3);
  canMsg1.data[5] = highByte(sensorValue4);
  canMsg1.data[6] = lowByte(sensorValue4);
  
  //message 3 "SystemInfo"
  //FuseTripped
  for (int i = 0; i <= 4; i++) {
    bitWrite(canMsg3.data[2],CHList[i].canControlSignalOffset, CHList[i].fusedTripped);
  }

  //Battery Voltage
  canMsg3.data[0] = highByte(sensorValue);
  canMsg3.data[1] = lowByte(sensorValue);
  Serial.println(canMsg3.data[2]);


  mcp2515.sendMessage(&canMsg1);
  mcp2515.sendMessage(&canMsg2);
  mcp2515.sendMessage(&canMsg3);


  if (interrupt) {
    interrupt = false;
    uint8_t irq = mcp2515.getInterrupts();

    if (irq & MCP2515::CANINTF_RX0IF) {
      if (mcp2515.readMessage(MCP2515::RXB0, &ctrlMsg) == MCP2515::ERROR_OK) {
        // frame contains received from RXB0 message
        ctrlMsgRecieved = true;
      }
    }
  }

  if (ctrlMsgRecieved) decodeCtrlMsg(ctrlMsg);

  delay(float(1) / UpdateRate * 1000);  //Crude wait, should use a dynamic time offset depending.
}

void decodeCtrlMsg(can_frame frame){
  ctrlMsgRecieved = false;
  for (int i = 0; i <= 4; i++) {
    if (!CHList[i].fusedTripped) { //fuse not tripped? Then send demand
      digitalWrite(CHList[i].SwitchOutputChannel, bitRead(frame.data[0],CHList[i].canControlSignalOffset));
    }else { //fuse tripped? only listen for fuse reset demand
      CHList[i].fusedTripped = bitRead(frame.data[0],CHList[i].canFuseTrippedOffset);
    }
  }

}