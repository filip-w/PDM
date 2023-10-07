#include <SPI.h>
#include "Adafruit_MAX31855.h"
#include <mcp2515.h>
#include <FIR.h>

//General setup
#define BaseCanID 0x500
#define UpdateRate 10 //Frontend thermocouple scan rate and CAN-message update rate in Hz.
#define serialDebug false
#define defaultOutputState HIGH

struct thermocoupleChannel{
   int channelNumber;
   Adafruit_MAX31855 thermocoupleFrontend;
   FIR<double, 10> fir_avg;
};

struct can_frame canMsg1;
enum CAN_SPEED BaudRate;
// For a moving average we use all ones as coefficients.
double coef_avg[10] = {1., 1., 1., 1., 1., 1., 1., 1., 1., 1.};
byte Lowpass;


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

MCP2515 mcp2515(CAN_ChipSelect);

void setup() {

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

  //Setup digital output pins
  pinMode(IN_CHN1, OUTPUT);
  pinMode(IN_CHN2, OUTPUT);
  pinMode(IN_CHN3, OUTPUT);
  pinMode(IN_CHN4, OUTPUT);
  digitalWrite(IN_CHN1, defaultOutputState);
  digitalWrite(IN_CHN2, defaultOutputState);
  digitalWrite(IN_CHN3, defaultOutputState);
  digitalWrite(IN_CHN4, defaultOutputState);
}

void loop() {
  mcp2515.sendMessage(&canMsg1);
  //mcp2515.sendMessage(&canMsg2);
  delay(float(1)/UpdateRate*1000); //Crude wait, should use a dynamic time offset depending.
}