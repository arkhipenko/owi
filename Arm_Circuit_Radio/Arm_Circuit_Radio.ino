// Arm controller v3.0.0

// Changelog:
//  2015-05-19:
//    v1.3.0 - compiled with a new TaskScheduler version 1.4 (sleep IDLE instead of delay)
//    v1.3.0 - buttons moved to pin state change interrupts instead of polling
//  2015-05-29:
//    v2.0.0 - expanded communication structure to transmit speed as well as direction
//  2015-12-01:
//    v2.0.1 - swtiched to EnableInterrupt library instead of PinChangeInt
//  2019-09-04:
//    v3.0.0 - migrated to github

// INCLUDES

#include <DirectIO.h>

#define EI_ARDUINO_INTERRUPTED_PIN
#include <EnableInterrupt.h>
//#include <PinChangeInt.h>

#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

#define _TASK_SLEEP_ON_IDLE_RUN
#include <TaskScheduler.h>

// TEST AND DEBUG DEFINES 

//#define _DEBUG_
//#define _TEST_


// DEFINES

#define  CE_PIN   8
#define  CSN_PIN  7

#define  PWM_M1_MIN  140
#define  PWM_M1_MAX  140

#define  PWM_M2_MIN  100
#define  PWM_M2_MAX  200

#define  PWM_M3_MIN  100
#define  PWM_M3_MAX  200

#define  PWM_M4_MIN  100
#define  PWM_M4_MAX  200

#define  PWM_M5_MIN  100
#define  PWM_M5_MAX  200

#define  GO_PIN       A3
#define  STOP_PIN     A2

Output<A0>            pLed;
Output<A1>            pSS1;
Input<STOP_PIN>       pStopButton;
Input<GO_PIN>         pGoButton;
Output<A4>            pBaseRight;
Output<A5>            pCommsLed;
AnalogOutput<3>       pBasePwm=0;
Output<4>             pBaseLeft;
AnalogOutput<5>       pArmPwm=0;
AnalogOutput<6>       pElbowPwm=0;
AnalogOutput<9>       pWristPwm=0;
AnalogOutput<10>      pClawPwm=0;


#define M_OFF      0b00
#define M_ON       0b11
#define M_RIGHT    0b01
#define M_LEFT     0b10
#define M_PWM_MIN  0
#define M_PWM_MAX  31

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
} ctrlWord, cwRx, cwPrev, cwActive;

struct {
  int m1;
  int m2;
  int m3;
  int m4;
  int m5;
} pwms;

// TASK DEFINES

#define RADIO_PERIOD    500   // radio -event driven via interrupts

#ifndef _DEBUG_
#define RADIO_TIMEOUT   2000   // 2 seconds
#else
#define RADIO_TIMEOUT   10000   // 10 seconds for debugging
#endif 

#define CTRL_PERIOD      1000   // 1 times/sec
#define MCTRL_PERIOD     1000   // normally speed increases in appropriate steps every N second(s)
#define MCTRL_REV_PER      60   // for one-time reverse we need to give full speed reverse for N ms
//#define BUTTON_PERIOD   100 


// TASKS

Scheduler runner;

// Callback methods prototypes:
void radioCallback();
void radioTimeoutCallback();
void armControlCallback();
void m1SpeedControlRevStop();
void m2SpeedControlRevStop();
void m3SpeedControlRevStop();
void m4SpeedControlRevStop();
void m5SpeedControlRevStop();
void greenButtonCallback();
void redButtonCallback();
void stopMotors();
void controlMotors();
void halt();
bool cwEqual(struct packed_bits& a, struct packed_bits& b);
void csn(int mode);

// Tasks:
Task tRadio (RADIO_PERIOD, TASK_FOREVER, &radioCallback);
Task tRadioTimeout (RADIO_TIMEOUT, TASK_FOREVER, &radioTimeoutCallback);
Task tArmControl (CTRL_PERIOD, TASK_FOREVER, &armControlCallback);
Task tM1Control (MCTRL_REV_PER, TASK_ONCE, &m1SpeedControlRevStop);
Task tM2Control (MCTRL_REV_PER, TASK_ONCE, &m2SpeedControlRevStop);
Task tM3Control (MCTRL_REV_PER, TASK_ONCE, &m3SpeedControlRevStop);
Task tM4Control (MCTRL_REV_PER, TASK_ONCE, &m4SpeedControlRevStop);
Task tM5Control (MCTRL_REV_PER, TASK_ONCE, &m5SpeedControlRevStop);
Task tGreenButton (TASK_IMMEDIATE, TASK_ONCE, &greenButtonCallback);
Task tRedButton (TASK_IMMEDIATE, TASK_ONCE, &redButtonCallback);

#ifdef _DEBUG_
void displayCallback();
Task tDisplay (1000, TASK_FOREVER, &displayCallback);
#endif


// VARIABLES

const uint64_t pipe = 0xE8E8F0F0E1LL; // Define the transmit pipe
volatile bool dsr;  // data set ready

// DRIVERS

RF24 radio(CE_PIN, CSN_PIN);


// CODE: CALLBACKS

void radioCallback() {
#ifdef _DEBUG_
//  Serial.print("radioCallback: dsr="); 
//  Serial.print(dsr); 
//  Serial.print(", radio.avail?=");
//  Serial.print(radio.available());
//  Serial.println();
#endif

  pCommsLed = LOW;
  if (dsr) {
      noInterrupts();
      ctrlWord = cwRx;
      dsr = false;
      interrupts();
      tArmControl.enable();
  }
}


void radioTimeoutCallback() {
#ifdef _DEBUG_
  Serial.println("Radio timeout!");
#endif
  stopMotors();
  halt();
}


void armControlCallback() {
  
  bool changed = false;

#ifdef _DEBUG_
//  Serial.print("armControlCallback"); 
//  Serial.println();
#endif
  
// Only do anything if changes were requested  
// LED - just execute the change immediately

// For motors - two different approaches:
// 1. If motor was stopped - start execution in the requested direction with gradual increase of speed
// 2. If the motor was moving and is requested to stop - initiate a momentary reverse pulse to break
// 3. If the motor was moving in one direction and is immediately requested to reverse do #2, then #1

  if ( !cwEqual(cwPrev, ctrlWord) ) {
    
    if (cwPrev.led != ctrlWord.led) {
      cwActive.led = ctrlWord.led;
      changed = true;
    }
    
    if (cwPrev.m1 != ctrlWord.m1) {
      if (ctrlWord.m1 == M_OFF) {
        cwActive.m1 = ~cwPrev.m1;  // reverse direction  and stop
        pwms.m1 = PWM_M1_MAX;
        tM1Control.restartDelayed();
      }
      else  {
        cwActive.m1 = ctrlWord.m1; 
        pwms.m1 = PWM_M1_MIN;
      }
      changed = true;
    }

    
    
    if (cwPrev.m2 != ctrlWord.m2) {
      if (ctrlWord.m2 == M_OFF) {
        cwActive.m2 = ~cwPrev.m2;  // reverse direction and stop
        pwms.m2 = PWM_M2_MAX;
        tM2Control.restartDelayed();
      }
      else {
        cwActive.m2 = ctrlWord.m2;  // reverse direction
        pwms.m2 = map(ctrlWord.pwm2, M_PWM_MIN, M_PWM_MAX, PWM_M2_MIN, PWM_M2_MAX); 

      }
      changed = true;
    }
    if (cwPrev.pwm2 != ctrlWord.pwm2) {
      pwms.m2 = map(ctrlWord.pwm2, M_PWM_MIN, M_PWM_MAX, PWM_M2_MIN, PWM_M2_MAX);
      changed = true;
    }
 
 
    
    if (cwPrev.m3 != ctrlWord.m3) {
      if (ctrlWord.m3 == M_OFF) {
        cwActive.m3 = ~cwPrev.m3;  // reverse direction and stop
        pwms.m3 = PWM_M3_MAX;
        tM3Control.restartDelayed();
      }
      else {
        cwActive.m3 = ctrlWord.m3;  // reverse direction
        pwms.m3 = map(ctrlWord.pwm3, M_PWM_MIN, M_PWM_MAX, PWM_M3_MIN, PWM_M3_MAX); 
      }
      changed = true;
    }
    if (cwPrev.pwm3 != ctrlWord.pwm3) {
      pwms.m3 = map(ctrlWord.pwm3, M_PWM_MIN, M_PWM_MAX, PWM_M3_MIN, PWM_M3_MAX);
      changed = true;
    }
 
 
    
    if (cwPrev.m4 != ctrlWord.m4) {
      if (ctrlWord.m4 == M_OFF) {
        cwActive.m4 = ~cwPrev.m4;  // reverse direction and stop
        pwms.m4 = PWM_M4_MAX;
        tM4Control.restartDelayed();
      }
       else {
        cwActive.m4 = ctrlWord.m4;  // reverse direction
        pwms.m4 = map(ctrlWord.pwm4, M_PWM_MIN, M_PWM_MAX, PWM_M4_MIN, PWM_M4_MAX);
      }
      changed = true;
    }
    if (cwPrev.pwm4 != ctrlWord.pwm4) {
      pwms.m4 = map(ctrlWord.pwm4, M_PWM_MIN, M_PWM_MAX, PWM_M4_MIN, PWM_M4_MAX);
      changed = true;
    }
    
    
    if (cwPrev.m5 != ctrlWord.m5) {
      if (ctrlWord.m5 == M_OFF) {
        cwActive.m5 = ~cwPrev.m5;  // reverse direction and stop
        pwms.m5 = PWM_M5_MAX;
        tM5Control.restartDelayed();
      }
       else  {
        cwActive.m5 = ctrlWord.m5;  // reverse direction
        pwms.m5 = map(ctrlWord.pwm5, M_PWM_MIN, M_PWM_MAX, PWM_M5_MIN, PWM_M5_MAX);
      }
      changed = true;
    }
    if (cwPrev.pwm5 != ctrlWord.pwm5) {
      pwms.m5 = map(ctrlWord.pwm5, M_PWM_MIN, M_PWM_MAX, PWM_M5_MIN, PWM_M5_MAX);
      changed = true;
    }
    
    
    cwPrev = ctrlWord;
    if (changed) controlMotors();
  }
}



void controlMotors() {
  
  byte *p = (byte *) &cwActive;
 
  if (cwActive.led == M_ON) pLed = HIGH;
  else pLed = LOW;

  if (cwActive.m1 == M_OFF) pwms.m1 = 0;
  if (cwActive.m2 == M_OFF) pwms.m2 = 0;
  if (cwActive.m3 == M_OFF) pwms.m3 = 0;
  if (cwActive.m4 == M_OFF) pwms.m4 = 0;
  if (cwActive.m5 == M_OFF) pwms.m5 = 0;
  else {
    pBaseLeft = (cwActive.m5 == M_LEFT);
    pBaseRight = (cwActive.m5 == M_RIGHT);
  }  
  
  pClawPwm = pwms.m1;
  pWristPwm = pwms.m2;
  pElbowPwm = pwms.m3;
  pArmPwm = pwms.m4;
  pBasePwm = pwms.m5;
  
// Shift out pin assignments to the shift register via SPI
  csn(LOW);
  SPI.transfer(*p);
  csn(HIGH);
}

void csn(int mode)
{
  // Minimum ideal SPI bus speed is 2x data rate
  // If we assume 2Mbs data rate and 16Mhz clock, a
  // divider of 4 is the minimum we want.
  // CLK:BUS 8Mhz:2Mhz, 16Mhz:4Mhz, or 20Mhz:5Mhz
  if (mode == LOW) {
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE0);
    SPI.setClockDivider(SPI_CLOCK_DIV4);
  }
  pSS1 = mode; 
}



void m1SpeedControlRevStop() {
  cwActive.m1 = M_OFF;
  pwms.m1 = 0;
  controlMotors();
}


void m2SpeedControlRevStop() {
  cwActive.m2 = M_OFF;
  pwms.m2 = 0;
  controlMotors();
}


void m3SpeedControlRevStop() {
  cwActive.m3 = M_OFF;
  pwms.m3 = 0;
  controlMotors();
}


void m4SpeedControlRevStop() {
  cwActive.m4 = M_OFF;
  pwms.m4 = 0;
  controlMotors();
}


void m5SpeedControlRevStop() {
  cwActive.m5 = M_OFF;
  pwms.m5 = 0;
  controlMotors();
}


//#define  GO_PIN      A3
//#define  STOP_PIN  A2

void initButtons() {
//  int buttonPressed=PCintPort::arduinoPin;
  int buttonPressed=arduinoInterruptedPin;
  if (buttonPressed == GO_PIN) tGreenButton.restart();
  if (buttonPressed == STOP_PIN) tRedButton.restart();
}

void redButtonCallback() {
#ifdef _DEBUG_
  Serial.println("redButtonCallback");
#endif
    halt();
}

void greenButtonCallback() {
#ifdef _DEBUG_
  Serial.println("greenButtonCallback");
#endif
    restart();
}


#ifdef _DEBUG_
void displayCallback() {
  char  ln[256];
  byte *p;
  
  p = (byte *) &ctrlWord;
  
  sprintf(ln, "led=%02d\tm1=%02d\tm2=%02d\tm3=%02d\tm4=%02d\tm5=%02d",ctrlWord.led,ctrlWord.m1,ctrlWord.m2,ctrlWord.m3,ctrlWord.m4,ctrlWord.m5);  
  Serial.println(ln);
  Serial.print(*p, HEX); p++; Serial.print(":"); Serial.println(*p, HEX);
}
#endif


// CODE: MAIN

void initPins() {
  pSS1 = HIGH;
  pinMode(2, INPUT);
  pinMode(3, OUTPUT); 
  pinMode(5, OUTPUT); 
  pinMode(6, OUTPUT); 
  pinMode(9, OUTPUT); 
  pinMode(10, OUTPUT);  // they say this is needed for SPI to be a master
//  pinMode(A4, OUTPUT);
}


void stopMotors() {

  initCtrlWord();
  cwRx = ctrlWord;
  cwPrev = ctrlWord;
  cwActive = ctrlWord;

  pwms.m1 = 0;
  pwms.m2 = 0;
  pwms.m3 = 0;
  pwms.m4 = 0;
  pwms.m5 = 0;
  
  controlMotors();
}

void initCtrlWord() {
  ctrlWord.led = M_OFF;
  ctrlWord.m1 = M_OFF;
  ctrlWord.m2 = M_OFF;
  ctrlWord.m3 = M_OFF;
  ctrlWord.m4 = M_OFF;
  ctrlWord.m5 = M_OFF;
  ctrlWord.pwm2 = 0;
  ctrlWord.pwm3 = 0;
  ctrlWord.pwm4 = 0;
  ctrlWord.pwm5 = 0;
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
//  radio.testCarrier();
  delay(10);
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


void restart() {
#ifdef _DEBUG_
  Serial.println("restart");
#endif  
  
  runner.disableAll();
  tRadio.enable();
  tRadioTimeout.enableDelayed();
  tArmControl.enable();
  tRedButton.enable();

  radio.startListening();

//  PCintPort::detachInterrupt(GO_PIN);
//  PCintPort::attachInterrupt(STOP_PIN, &initButtons, RISING); 
  disableInterrupt(GO_PIN);
  enableInterrupt(STOP_PIN, &initButtons, RISING); 
  
#ifdef _DEBUG_
  runner.addTask(tDisplay);
  tDisplay.enable();
#endif
}


void halt() {
#ifdef _DEBUG_
  Serial.println("halt");
#endif  

  stopMotors();
  runner.disableAll();
  tGreenButton.enable();
  radio.powerDown();
  pCommsLed = LOW;

//  PCintPort::detachInterrupt(STOP_PIN);
//  PCintPort::attachInterrupt(GO_PIN, &initButtons, RISING); 
  disableInterrupt(STOP_PIN);
  enableInterrupt(GO_PIN, &initButtons, RISING); 
}


void setup() {
  initPins();
  
  SPI.begin();
  stopMotors();
  
#ifdef _DEBUG_
  Serial.begin(115200);
  Serial.println("Robotic Arm Motor Controller Circuit test");
#endif

#ifndef _DEBUG_
  power_adc_disable();
  power_twi_disable();
  power_usart0_disable();
#endif

  radio.begin();
  initRadio();
  radio.openReadingPipe(1, pipe);
  
  dsr = false;
//  attachInterrupt(0, check_radio, FALLING);
  enableInterrupt(0, &check_radio, FALLING);
  
  runner.init();
  
  runner.addTask(tRadio);
  runner.addTask(tRadioTimeout);
  runner.addTask(tArmControl);
  runner.addTask(tM1Control);
  runner.addTask(tM2Control);
  runner.addTask(tM3Control);
  runner.addTask(tM4Control);
  runner.addTask(tM5Control);
  runner.addTask(tGreenButton);
  runner.addTask(tRedButton);

  halt();
}


void loop() {
  runner.execute();
}


void check_radio(void)
{
  // What happened?
  bool tx,fail,rx;
  radio.whatHappened(tx,fail,rx);

  if ( rx )
  {
    dsr = true;
    radio.read( (byte*) &cwRx, sizeof(cwRx) );
    pCommsLed = HIGH;
    tRadioTimeout.delay();
    tRadio.enable();
  }
}
