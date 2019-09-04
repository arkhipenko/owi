// Glove controller v3.0.0
//
// Changelog:
//  2015-05-19:
//    v1.3.0 - slower comms blinking (to be noticeable)
//    v1.3.0 - compiled with a new TaskScheduler version 1.4 (sleep IDLE instead of delay)
//  2015-05-20:
//    v1.3.1 - change base rotation control to be accel driven as well
//    v1.3.1 - complied against new TaskScheduler with IDLE sleep
//  2015-05-27:
//    v1.4.0 - gradual control for all motors except gripper
//    v1.4.0 - new gestures: gripper: buttons;  wrist: wrist tilt up/down; elbow: palm rotating right/left;  shoulder: palm straight up then rotating right/left;
//             base: palm straight down then rotating right/left;
//  2019-09-04:
//    v3.0.0. - migrated to github

// INCLUDES

#include <DirectIO.h>

#include <I2Cdev.h>
#include <Wire.h>
#include <MPU6050.h>
#include <AvgFilter.h>
#include <DhpFilter.h>

#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

#define _TASK_SLEEP_ON_IDLE_RUN
#include <TaskScheduler.h>


// TEST AND DEBUG DEFINES

//#define _DEBUG_
//#define _TEST_


// DEFINES

#define  CLOSE_CLAW_BUTTON  4
#define  OPEN_CLAW_BUTTON   3
#define  FLEX_SENSOR       A0
#define  LED               A1
#define  CE_PIN             9
#define  CSN_PIN           10

Input<CLOSE_CLAW_BUTTON> pCloseClaw;
Input<OPEN_CLAW_BUTTON>  pOpenClaw;
Output<LED>              pLed;
AnalogInput<FLEX_SENSOR> pFlex;

#define M_OFF      0b00
#define M_ON       0b11
#define M_RIGHT    0b01
#define M_LEFT     0b10

#define M_PWM_MIN  0
#define M_PWM_MAX  32

//#define ACCON_TH  9000
//#define ACCOFF_TH 8000

//#define FLIP_TH  (-3000)

#define  RADIO_DEFAULT_CHANNEL  56

// STRUCTURES

struct packed_bits {
  byte m1   : 2; // gripper
  byte m2   : 2; // wrist
  byte m3   : 2; // elbow
  byte m4   : 2; // shoulder
  byte m5   : 2; // base rotate
  byte led  : 2; // gripper LED
  byte pwm2 : 5; // wrist
  byte pwm3 : 5; // elbow
  byte pwm4 : 5; // shoulder
  byte pwm5 : 5; // base
} ctrlWord, cwTx, cwPrev;


// TASK DEFINES

#define MEASURE_PERIOD  50   // 20 times/sec
#define TRANSMIT_PERIOD 100   // 2 times/sec or on demand
#define TRAN_LED_PERIOD 200  // 5 times/sec
#define REACTION_DELAY  100   //

#define GYRO_TH   4000
#define LEVEL_TH  4000
#define VERT_TH    13400 // ~ (16383 - LEVEL_TH)

#define FLEX_OPN_ON   470
#define FLEX_OPN_OFF  430
#define FLEX_CLS_ON   410
#define FLEX_CLS_OFF  370

#define WRIST_MIN     6000
#define WRIST_MAX    14000
#define WRIST_OFF     3000   // for AZ

#define ELBOW_MIN     6000
#define ELBOW_MAX    14000

#define SHLDR_MIN     6000
#define SHLDR_MAX    14000

#define BASE_MIN     6000
#define BASE_MAX    14000


// TASKS

Scheduler runner;
// Callback functions prototypes:
void measureCallback();
void flexDelayCallback();
void transmitCallback();
bool cwEqual(struct packed_bits& a, struct packed_bits& b);


// Tasks:
Task tMeasure(MEASURE_PERIOD, -1, &measureCallback);
Task tFlexDelay(REACTION_DELAY, 1, &flexDelayCallback);
Task tTransmit(TRANSMIT_PERIOD, -1, &transmitCallback);

#ifdef _DEBUG_
void displayCallback();
Task tDisplay(500, -1, &displayCallback);
#endif


// VARIABLES

const uint64_t pipe = 0xE8E8F0F0E1LL; // Define the transmit pipe
volatile bool rts;
volatile bool radio_status;

// Glove controls/sensors


// DRIVERS

MPU6050 ag;
RF24 radio(CE_PIN, CSN_PIN);


// CODE: CALLBACKS

int b1, b2, flex, ax, ay, az, gx, gy, gz;
long axFd[3], ayFd[3], azFd[3], flexFd[3];
avgFilter axF(3, axFd), ayF(3, ayFd), azF(3, azFd), flexF(3, flexFd);
dhpFilter gxF, gyF, gzF;

void measureCallback() {

  bool changed = false;

  cwPrev = ctrlWord;

  ag.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  ax = axF.value(ax);
  ay = ayF.value(ay);
  az = azF.value(az);
  gx = gxF.value(gx);
  gy = gyF.value(gy);
  //  gz = gzF.value(gz);

  b1 = pCloseClaw;
  b2 = pOpenClaw;


  //  Buttons:
  //    B1 (middle) - LED on
  //    B2 (pinky)  - LED off

  if (b1) {
    ctrlWord.led = M_ON;
  }

  if (b2) {
    ctrlWord.led = M_OFF;
  }

   // Flex: Gripper
   
  flex = flexF.value(pFlex);
  if (flex > FLEX_OPN_ON || flex < FLEX_CLS_OFF) {
    ctrlWord.m1 = M_OFF;
    tFlexDelay.disable();
  }
  else {
    if ( !tFlexDelay.isEnabled() ) tFlexDelay.restartDelayed();
  }


    // Wrist
  
// Filtering condition: palm level (X axis pointing forward) rotating over Y axis within X bounadies)
  if ( abs(ay) < LEVEL_TH && abs(ax) >= WRIST_MIN && abs(ax) <= WRIST_MAX ) {
    if (abs(gy) < GYRO_TH) {
       ctrlWord.m2 = ax >0 ? M_RIGHT : M_LEFT;
       ctrlWord.pwm2 = map (abs(ax), WRIST_MIN, WRIST_MAX, M_PWM_MIN, M_PWM_MAX);
    }
  }
  else {
    ctrlWord.m2 = M_OFF;
    ctrlWord.pwm2 = 0;
  }


    // Elbow
    
// Filtering condition: palm level (X axis pointing forward) rotating over X axis within Y bounadies)
  if ( abs(ax) < LEVEL_TH && abs(ay) >= ELBOW_MIN && abs(ay) <= ELBOW_MAX ) {
    if (abs(gx) < GYRO_TH) {
       ctrlWord.m3 = ay >0 ? M_RIGHT : M_LEFT;
       ctrlWord.pwm3 = map (abs(ay), ELBOW_MIN, ELBOW_MAX, M_PWM_MIN, M_PWM_MAX);
    }
  }
  else {
    ctrlWord.m3 = M_OFF;
    ctrlWord.pwm3 = 0;
  }


  // Shoulder
  
// Filtering condition: palm up (X axis pointing up) + rotating over Z axis within Y bounadies)
  if ( abs(az) < LEVEL_TH && ax > 0 && abs(ay) >= SHLDR_MIN && abs(ay) <= SHLDR_MAX ) {
     ctrlWord.m4 = ay >0 ? M_RIGHT : M_LEFT;
     ctrlWord.pwm4 = map (abs(ay), SHLDR_MIN, SHLDR_MAX, M_PWM_MIN, M_PWM_MAX);
  }
  else {
    ctrlWord.m4 = M_OFF;
    ctrlWord.pwm4 = 0;
  }



  // Base
  
// Filtering condition: palm down (X axis pointing down) + rotating over Z axis within Y bounadies)
  if ( abs(az) < LEVEL_TH && ax < 0 && abs(ay) >= BASE_MIN && abs(ay) <= BASE_MAX ) {
     ctrlWord.m5 = ay <0 ? M_LEFT : M_RIGHT;
     ctrlWord.pwm5 = map (abs(ay), BASE_MIN, BASE_MAX, M_PWM_MIN, M_PWM_MAX);
  }
  else {
    ctrlWord.m5 = M_OFF;
    ctrlWord.pwm5 = 0;
  }


  if ( !cwEqual(cwPrev, ctrlWord) ) tTransmit.enable();
}



void flexDelayCallback() {
  flex = flexF.value(pFlex);

  if (flex <= FLEX_OPN_ON && flex >= FLEX_OPN_OFF) {
    ctrlWord.m1 = M_RIGHT;
    tTransmit.enable();
  }

  if (flex <= FLEX_CLS_ON && flex >= FLEX_CLS_OFF) {
    ctrlWord.m1 = M_LEFT;
    tTransmit.enable();
  }
  tFlexDelay.disable();
}



void transmitCallback() {

  if (!rts) return;

 // noInterrupts();
  cwTx = ctrlWord;
  rts = false;
 // interrupts();

  if (radio_status) pLed = HIGH;
  radio.startWrite( &cwTx, sizeof(cwTx) );
}


#ifdef _DEBUG_
void displayCallback() {
  char  ln[256];
  byte *p;

  p = (byte *) &ctrlWord;

  sprintf(ln, "led=%02d\tm1=%02d\tm2=%02d\tm3=%02d\tm4=%02d\tm5=%02d\tflex=%04d", ctrlWord.led, ctrlWord.m1, ctrlWord.m2, ctrlWord.m3, ctrlWord.m4, ctrlWord.m5, flex);
  Serial.println(ln);
  sprintf(ln, "pwm2=%02d\tpwm3=%02d\tpwm4=%02d\tpwm5=%02d", ctrlWord.pwm2, ctrlWord.pwm3, ctrlWord.pwm4, ctrlWord.pwm5);
  Serial.println(ln);
  Serial.print(*p, HEX); p++; Serial.print(":"); Serial.println(*p, HEX);
}
#endif



// CODE: MAIN
void initCtrlWord() {
  ctrlWord.led = M_OFF;
  ctrlWord.m1 = M_OFF;
  ctrlWord.m2 = M_OFF;
  ctrlWord.m3 = M_OFF;
  ctrlWord.m4 = M_OFF;
  ctrlWord.m5 = M_OFF;
}


void initPins() {
  //  pinMode(CLOSE_CLAW_BUTTON, INPUT);
  //  pinMode(OPEN_CLAW_BUTTON, INPUT);
  //  pinMode(FLEX_SENSOR, INPUT);
  //  pinMode(LED, OUTPUT);
  pinMode(10, OUTPUT); // SPI needs it?
}


void initRadio() {

  //  radio.setChannel(RADIO_DEFAULT_CHANNEL);  // 0-127
  radio.setDataRate(RF24_250KBPS);  // RF24_1MBPS = 0, RF24_2MBPS, RF24_250KBPS
  //  radio.setRetries(5,5);
  radio.setPayloadSize( sizeof(ctrlWord) );
  //  radio.setAutoAck(true);
  //  radio.enableAckPayload();
  radio.setCRCLength(RF24_CRC_8);  // RF24_CRC_DISABLED = 0, RF24_CRC_8, RF24_CRC_16
  //  radio.setPALevel(RF24_PA_HIGH); // RF24_PA_MIN=-18dBm, RF24_PA_LOW=-12dBm, RF24_PA_MED=-6dBM, and RF24_PA_HIGH=0dBm. *  RF24_PA_MIN = 0,RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX, RF24_PA_ERROR
  delay(10);
  //  radio.testCarrier();
}



void setup() {
  // put your setup code here, to run once:
  initPins();
  initCtrlWord();

  Wire.begin();       //Initiate the Wire library and join the I2C bus as a master

#ifdef _DEBUG_
  Serial.begin(115200);
  Serial.println("Robotic Arm Glove Controller Circuit test");
#endif

#ifndef _DEBUG_
  power_usart0_disable();
#endif

  // Configure radio
  radio.begin();
  initRadio();

  ag.initialize();
  if (!ag.testConnection()) {
#ifdef _DEBUG_
    Serial.println("Accelerometer is not responding");
#endif
    while (1) {};
  }

  attachInterrupt(0, check_radio, FALLING);
  rts = true;

  runner.init();

  runner.addTask(tMeasure);
  //  runner.addTask(tLedDelay);
  //  runner.addTask(tB1Delay);
  //  runner.addTask(tB2Delay);
  runner.addTask(tFlexDelay);
  runner.addTask(tTransmit);

#ifdef _DEBUG_
  runner.addTask(tDisplay);
  tDisplay.enable();
#endif

  tMeasure.enable();
  tTransmit.enable();

  for (int i = 1; i < 6; i++) {
    pLed = i & 1 ? HIGH : LOW;
    delay(400);
  }



  // Start transmitting
  radio.powerUp();

  for (int i = 0; i < 12; i++) {
    pLed = i & 1 ? HIGH : LOW;
    delay(200);
  }

  radio.openWritingPipe(pipe);
}


void loop() {
  runner.execute();
}


void check_radio(void)
{
  // What happened?
  bool tx, fail, rx;
  radio.whatHappened(tx, fail, rx);

  pLed = LOW;
  radio_status = false;
  // Have we successfully transmitted?
  if ( tx )
  {
    radio_status = true;
  }

  rts = true;
}

bool cwEqual(struct packed_bits& a, struct packed_bits& b) {
  byte *aByte = (byte *) &a;
  byte *bByte = (byte *) &b;
  bool eq = true;
  int sz = sizeof(a);

  for (int i = 0; i < sz && eq; i++) {
    eq = (*aByte++ == *bByte++);
  }

  return eq;
  //  return ( (a.led == b.led) && (a.m1 == b.m1) && (a.m2 == b.m2) && (a.m3 == b.m3) && (a.m4 == b.m4) && (a.m5 == b.m5) && (a.pwm2 == b.pwm2) && (a.pwm3 == b.pwm3) && (a.pwm4 == b.pwm4) && (a.pwm5 == b.pwm5) );
}

