/*
 * CSCE 462
 *
 * Split Flap Display
 * 
 * Authors: Caroline Mejia, Jack Warsham, Chris Weeks
 */

// Includes

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Preprocessor Definitions

#define LOOP_TIME 5 // 5ms

// Split Flap Info
#define NUM_MODULES 12
#define NUM_FLAPS 27

// somewhat arbitrarily set number of increments per revolution to 1000 * NUM_FLAPS.
#define COUNTS_PER_REV (1000 * NUM_FLAPS)
#define COUNTS_PER_FLAP (COUNTS_PER_REV / NUM_FLAPS)

// PWM Driver
#define PWM_CTRL_ADDR 0x40
// ideally, the pwm driver clk freq should be 25MHz but it is not exactly and must be calibrated
#define PWM_CLK_FREQ 24750000
#define PWM_FREQ 50

// Servos
// pulse lengths out of 4096
#define SERVO_ON_PULSE_LENGTH_FAST 321
#define SERVO_ON_PULSE_LENGTH_SLOW 314
#define SERVO_OFF_PULSE_LENGTH 300
#define SERVO_RPM_FAST 72.00f // todo: calibrate this. May  need to calibrate for each servo
#define SERVO_RPM_SLOW 59.40f
// how far does servo move when on, measured in counts per loop
#define SERVO_DELTA_OMEGA_FAST (unsigned int)(SERVO_RPM_FAST * LOOP_TIME * COUNTS_PER_REV / 60000)
#define SERVO_DELTA_OMEGA_SLOW (unsigned int)(SERVO_RPM_SLOW * LOOP_TIME * COUNTS_PER_REV / 60000)
#define SERVO_DELTA_OMEGA(x) (unsigned int)(x * LOOP_TIME * COUNTS_PER_REV / 60000)

// Hall Effect Sensors
#define HE_TRIGGER_VOLTAGE 512 // TODO: calibrate this. What voltage to use to detect HE falling edge


enum ServoState
{
  E_OFF = 0,
  E_START,
  E_SLOW,
  E_FAST,
};

// Structs
typedef struct
{
    unsigned char adcID;                      // which ADC is the HE connected to. -1: Arduino built in
    unsigned char adcPin;            // which pin of the ADC is this HE connected to
    unsigned int HEPositionOffset;  // offset in counts for where HE and magnet cross relative to defined zero position (such as A or blank flap)
    unsigned short last_value;
    unsigned short value;
} HallEffect;

// typedef struct
// {
//     unsigned int pwmPin;            // which pwm pin this servo is connected to
//     unsigned int position;          // position mod COUNTS_PER_REV
//     unsigned int targetPosition;
//     ServoState state;
//     unsigned int startupCounter;
//     float targetRPM;
// } Servo;

typedef struct
{
  unsigned short pulseLengthOff;
  unsigned short pulseLengthFast;
  unsigned short pusleLengthSlow;
  float rpmFast;
  float rpmSlow;
} ServoCal;

ServoCal servoCals[NUM_MODULES] = 
{
  {300, 321, 314, 72.00f, 59.40f},
  {295, 321, 310, 60.60f, 42.67f},
  {300, 321, 314, 72.00f, 59.40f},
  {300, 321, 314, 72.00f, 59.40f},
  {300, 321, 314, 72.00f, 59.40f},
  {300, 321, 314, 72.00f, 59.40f},
  {300, 321, 314, 72.00f, 59.40f},
  {300, 321, 314, 72.00f, 59.40f},
  {300, 321, 314, 72.00f, 59.40f},
  {300, 321, 314, 72.00f, 59.40f},
  {300, 321, 314, 72.00f, 59.40f},
  {300, 321, 314, 72.00f, 59.40f},
};

typedef struct
{
  unsigned int pwmPin;            // which pwm pin this servo is connected to
  unsigned int position;          // position mod COUNTS_PER_REV
  unsigned int targetPosition;
  ServoState state;
  unsigned int startPosition;
  float targetRPM;
  float lastTargetRpm;
  ServoCal *cal;
} Servo;

typedef struct
{
  HallEffect hallEffect;
  Servo servo;
} SFModule;

SFModule sfModules[NUM_MODULES] = 
{
    {{0, 0, 0, 0, 0}, {0,  0, 0, 0, 0, 0, 0, &servoCals[1]}},       // {HE{adcID, adcPin, offset, 0, 0}, Servo{pwmPin, 0, 0, 0, 0, 0, calAddr}}
    {{0, 1, 0, 0, 0}, {1,  0, 0, 0, 0, 0, 0, &servoCals[1]}},
    {{0, 2, 0, 0, 0}, {2,  0, 0, 0, 0, 0, 0, &servoCals[2]}},
    {{0, 3, 0, 0, 0}, {3,  0, 0, 0, 0, 0, 0, &servoCals[3]}},
    {{0, 4, 0, 0, 0}, {4,  0, 0, 0, 0, 0, 0, &servoCals[4]}},
    {{0, 5, 0, 0, 0}, {5,  0, 0, 0, 0, 0, 0, &servoCals[5]}},
    {{0, 6, 0, 0, 0}, {6,  0, 0, 0, 0, 0, 0, &servoCals[6]}},
    {{0, 7, 0, 0, 0}, {7,  0, 0, 0, 0, 0, 0, &servoCals[7]}},
    {{1, 0, 0, 0, 0}, {8,  0, 0, 0, 0, 0, 0, &servoCals[8]}},
    {{1, 1, 0, 0, 0}, {9,  0, 0, 0, 0, 0, 0, &servoCals[9]}},
    {{1, 2, 0, 0, 0}, {10, 0, 0, 0, 0, 0, 0, &servoCals[10]}},
    {{1, 3, 0, 0, 0}, {11, 0, 0, 0, 0, 0, 0, &servoCals[11]}},
};



// Global variables
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PWM_CTRL_ADDR);

char message[NUM_MODULES];  // 0=Blank. [1-26]=[A-Z]
bool newMessageReceived = false;
int debug = 0;

// Function Definitions
int calculatePWM(float rpm, ServoCal *cal)
{
  if(rpm == 0) return cal->pulseLengthOff;
  if(cal->rpmFast == cal->rpmSlow) return cal->pusleLengthSlow;

  return ((cal->pulseLengthFast - cal->pusleLengthSlow) / (cal->rpmFast - cal->rpmSlow)) * (rpm - cal->rpmSlow) + cal->pusleLengthSlow;
}


int ADC_Read(unsigned int adcId, unsigned int adcPin) {
  if(adcId == -1)
  {
    return analogRead(adcPin);
  }
  // TODO: 3008 read
  return 0.0;
}

void update() {
  SFModule *sfm;

  if(newMessageReceived)
  {
      for(int i = 0; i < NUM_MODULES; ++i)
      {
          // interpret period as 'module is behind by one, advance it by one letter without changing final position'
          if(message[i] == '.')
          {
            sfModules[i].servo.position = (sfModules[i].servo.position - COUNTS_PER_FLAP) % COUNTS_PER_REV;
            continue;
          }
          // interpret comma as 'module is ahead by one, loop around to get to previous letter'
          else if(message[i] == ',')
          {
            sfModules[i].servo.position = (sfModules[i].servo.position + COUNTS_PER_FLAP) % COUNTS_PER_REV;
            continue;
          }
          // interpret slash as do nothing
          else if(message[i] == '/')
          {
            continue;
          }
          // assuming blank flap is at position 0, target postion is cacluated as follows
          sfModules[i].servo.targetPosition = message[i] * COUNTS_PER_FLAP;
      }
      newMessageReceived = false; // UNCOMMENT
  }

  // for(int i = 0; i < NUM_MODULES; ++i)
  for(int i = 0; i < 1; ++i)
  {
    sfm = &sfModules[i];

    // update HE value
    sfm->hallEffect.last_value = sfm->hallEffect.value;
    sfm->hallEffect.value = ADC_Read(sfm->hallEffect.adcID, sfm->hallEffect.adcPin);
    
    // update position
    sfm->servo.position = (sfm->servo.position + SERVO_DELTA_OMEGA(sfm->servo.targetRPM)) % COUNTS_PER_REV;

    // check for HE falling edge
    if(sfm->hallEffect.last_value > HE_TRIGGER_VOLTAGE 
        && sfm->hallEffect.value < HE_TRIGGER_VOLTAGE)
    {
        sfm->servo.position = sfm->hallEffect.HEPositionOffset;
    }

    // calculate new target rpm
    sfm->servo.lastTargetRpm = sfm->servo.targetRPM;
    if(sfm->servo.position >= sfm->servo.targetPosition && sfm->servo.position < sfm->servo.targetPosition + (COUNTS_PER_FLAP / 4))
    {
      sfm->servo.targetRPM = 0;
      sfm->servo.state = E_OFF;
    }
    else
    {
      // startup
      // if(sfm->servo.state == E_OFF)
      // {
      //   sfm->servo.targetRPM = sfm->servo.cal->rpmFast;
      //   sfm->servo.startPosition = sfm->servo.position;
      //   sfm->servo.state = E_START;
      // }
      // else if(sfm->servo.position >= sfm->servo.startPosition && sfm->servo.position < sfm->servo.startPosition + (COUNTS_PER_FLAP * 2))
      // {
      //   sfm->servo.targetRPM = sfm->servo.cal->rpmFast;
      // }
      if(sfm->servo.state == E_OFF)
      {
        sfm->servo.targetRPM = sfm->servo.cal->rpmSlow;
        sfm->servo.state = E_START;
      }
      if(sfm->servo.state == E_START)
      {
        sfm->servo.targetRPM += 0.1;
        if(sfm->servo.targetRPM > sfm->servo.cal->rpmFast)
        {
          sfm->servo.targetRPM = sfm->servo.cal->rpmFast;
          sfm->servo.state = E_FAST;
        }
      }

      // slow down
      else if((sfm->servo.targetPosition - sfm->servo.position) % COUNTS_PER_REV < COUNTS_PER_FLAP * 8)
      {
        sfm->servo.targetRPM -= 0.1;
        if(sfm->servo.targetRPM < sfm->servo.cal->rpmSlow)
        {
          sfm->servo.targetRPM = sfm->servo.cal->rpmSlow;
        }
        sfm->servo.state = E_SLOW;
      }

      // middle
      else
      {
        sfm->servo.targetRPM = sfm->servo.cal->rpmFast;
        sfm->servo.state = E_FAST;
      }
    }

    // send updates to servo
    if(sfm->servo.targetRPM != sfm->servo.lastTargetRpm)
    {
      // Serial.print(sfm->servo.targetRPM);
      // Serial.print(", ");
      // Serial.println(calculatePWM(sfm->servo.targetRPM, sfm->servo.cal));
      interrupts();
      pwm.setPin(sfm->servo.pwmPin, calculatePWM(sfm->servo.targetRPM, sfm->servo.cal));
      noInterrupts();
    }
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  Wire.setClock(100000);
  Wire.begin();

  cli(); // disable interrupts

  // init timer 1 to run every 5ms
  // Sys Clk = 16Mhz
  // 16Mhz / 8 / 10000 = 200 Hz (5 ms period)
  TCCR1A = 0; // reset timer registers
  TCCR1B = 0;
  TCCR1B |= B00000010; // /8 prescalar
  TIMSK1 |= B00000010; // enable CMPA interrupt
  OCR1A = 10000; // interrupt when timer reaches 10000 ticks
  sei(); // enable interrupts


  pwm.begin();
  pwm.setOscillatorFrequency(PWM_CLK_FREQ);
  pwm.setPWMFreq(PWM_FREQ);

  delay(10);

  for(int i = 0; i < NUM_MODULES; ++i)
  {
    pwm.setPin(sfModules[i].servo.pwmPin, sfModules[i].servo.cal->pulseLengthOff);
  }
}

void GetNewMessage() {
  
  // do not process new message until old message has been processed
  if(newMessageReceived)
  {
    return;
  }

  char newMessage[NUM_MODULES] = {0};

  Serial.print("Enter string: ");
  while(Serial.available() == 0) {delay(5);} // wait for input
  delay(30); // give some time for full input to arrive
  int i = 0;
  char nextChar;
  while(Serial.available() > 0 && i < NUM_MODULES)
  {
    delay(10);
    nextChar = Serial.read();
    Serial.print(nextChar);
    // if(nextChar == '\n' || nextChar == '\0') {
    //   memset(&newMessage[i], 0, NUM_MODULES - i);
    //   break;
    // }
    if(nextChar >= 'A' && nextChar <= 'Z')
    {
        newMessage[i] = nextChar - 64;
    }
    else if(nextChar >= 'a' && nextChar <= 'z')
    {
        newMessage[i] = nextChar - 96;
    }
    else if(nextChar == ',' || nextChar == '.' || nextChar == '/')
    {
      // do nothing
      newMessage[i] = nextChar;
    }
    else
    {
        // either a proper space, or an invalid character, set to space for both
        newMessage[i] = 0;
    }
    i++;
  }
  // Serial.println(i);
  memset(&newMessage[i], 0, NUM_MODULES - i);
  // Serial.println("test");
  memcpy(message, newMessage, NUM_MODULES);
  Serial.println("test1");
  newMessageReceived = true;
}

void loop() {
  // put your main code here, to run repeatedly:
  GetNewMessage();
  Serial.println(sfModules[0].servo.position);
  // Serial.println(sfModules[0].servo.targetPosition);
  // Serial.println(sfModules[0].servo.isOn);
  // Serial.println(SERVO_DELTA_OMEGA_FAST);
  delay(1000);
}

// timer 1 interrupts runs ever 5 ms
ISR(TIMER1_COMPA_vect) {
  TCNT1 = 0; // reset timer to 0
  update();
}