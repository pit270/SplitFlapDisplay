/*
 * CSCE 462
 *
 * Split Flap Display with RGB backlight
 * 
 * Authors: Caroline Mejia, Jack Warsham, Chris Weeks
 */

// Includes
#include "WiringPi/wiringPi/wiringPi.h"
#include "WiringPi/wiringPi/wiringPiI2C.h"
#include <stdio.h>
#include <unistd.h>
#include "pca9685/src/pca9685.h"

// Preprocessor Definitions
#define LOOP_TIME_MS 1
#define LOOP_TIME_US (LOOP_TIME_MS * 1000)

#define NUM_MODULES 16
#define NUM_FLAPS 27

// somewhat arbitrarily set number of increments per revolution to 10 * NUM_FLAPS. This value can be changed later if needed
#define COUNTS_PER_REV (10 * NUM_FLAPS)
#define COUNTS_PER_FLAP (COUNTS_PER_REV / NUM_FLAPS)

#define SERVO_VELOCITY_RPM 110.0 // May need to calibrate each servo individually
#define SERVO_VELOCITY_LOOP (int)(SERVO_VELOCITY_RPM / 60.0 / 1000.0 * LOOP_TIME_MS * COUNTS_PER_REV) // Counts / loop
#define PWM_FREQ 50 //Hertz (20ms)
#define SERVO_ON_PULSE_WIDTH_MS 1.0f // ms
#define SERVO_OFF_PULSE_WIDTH_MS 0.0f // ms. 
#define SERVO_ON_PULSE_WIDTH (4096 * SERVO_ON_PULSE_WIDTH_MS / (1000.0 / PWM_FREQ) + 0.5f)
#define SERVO_OFF_PULSE_WIDTH (4096 * SERVO_OFF_PULSE_WIDTH_MS / (1000.0 / PWM_FREQ) + 0.5f)
#define PIN_BASE 300 //base for the virtual pin numbers for PWM
#define PWM_PIN(x) (PIN_BASE + x)

#define HE_TRIGGER_VOLTS 2.5    // TODO: calibrate this. what voltage to use to detect falling edge of HE

// I2C Addresses
#define PWM_CONTROLLER_ADDR 0x40

// PINOUTS

// Structs
typedef struct
{
    unsigned int adcID;             // which ADC is the HE connected to
    unsigned int adcPin;            // which pin of the ADC is this HE connected to
    unsigned int HEPositionOffset;  // offset in counts for where HE and magnet cross relative to defined zero position (such as A or blank flap)
    float last_value;
    float value;
} HallEffect;

typedef struct
{
    unsigned int pwmPin;            // which pwm pin this servo is connected to
    unsigned int position;          // position mod COUNTS_PER_REV
    unsigned int targetPosition;
    bool isOn;
} Servo;

typedef struct
{
    HallEffect hallEffect;
    Servo servo;
} SFModule;

SFModule sfModules[NUM_MODULES] = 
{
    {{0, 0, 0, 0.0f, 0.0f}, {0, 0, 0, 0}},       // {HE{adcID, adcPin, offset, 0, 0}, Servo{pwmPin, 0, 0, 0}}
    {{0, 1, 0, 0.0f, 0.0f}, {1, 0, 0, 0}},
    {{0, 2, 0, 0.0f, 0.0f}, {2, 0, 0, 0}},
    {{0, 3, 0, 0.0f, 0.0f}, {3, 0, 0, 0}},
    {{0, 4, 0, 0.0f, 0.0f}, {4, 0, 0, 0}},
    {{0, 5, 0, 0.0f, 0.0f}, {5, 0, 0, 0}},
    {{0, 6, 0, 0.0f, 0.0f}, {6, 0, 0, 0}},
    {{0, 7, 0, 0.0f, 0.0f}, {7, 0, 0, 0}},
    {{1, 0, 0, 0.0f, 0.0f}, {8, 0, 0, 0}},
    {{1, 1, 0, 0.0f, 0.0f}, {9, 0, 0, 0}},
    {{1, 2, 0, 0.0f, 0.0f}, {10, 0, 0, 0}},
    {{1, 3, 0, 0.0f, 0.0f}, {11, 0, 0, 0}},
    {{1, 4, 0, 0.0f, 0.0f}, {12, 0, 0, 0}},
    {{1, 5, 0, 0.0f, 0.0f}, {13, 0, 0, 0}},
    {{1, 6, 0, 0.0f, 0.0f}, {14, 0, 0, 0}},
    {{1, 7, 0, 0.0f, 0.0f}, {15, 0, 0, 0}},
}

// Global variables
int i2cFD = -1;

// volatile because right now assuming this may be modified by another process
volatile char message[NUM_MODULES] = {0};      // 0=Blank. [1-26]=[A-Z]
volatile bool newMessageReceived = false;

// TODO: ADC_Read
float ADC_Read(adcID, adcPin)
{
    return 0.0f;
}

// Function Definitions
void Update()
{
    SFModule *sfm;

    if(newMessageReceived)
    {
        for(int i = 0; i < NUM_MODULES; ++i)
        {
            // assuming blank flap is at position 0, target postion is cacluated as follows
            sfModules[i].servo.targetPosition = message[i] * COUNTS_PER_FLAP;
        }
        newMessageReceived = false;
    }

    for(int i = 0; i < NUM_MODULES; ++i)
    {
        sfm = &sfModules[i];
        // update HE value
        sfm->hallEffect.last_value = sfm->hallEffect.value;
        sfm->hallEffect.value = ADC_Read(sfm->hallEffect.adcID, sfm->hallEffect.adcPin);
        // update servo position and time calculations and servo status
        if(sfm->servo.position > sfm->servo.targetPosition && sfm->servo.position < sfm->servo.targetPosition + (COUNTS_PER_FLAP / 4))
        {
            // stop servo
            pwmWrite(PWM_PIN(sfm->servo.pwmPin), SERVO_OFF_PULSE_WIDTH);
            sfm->servo.isOn = false;
        }
        else
        {
            // to avoid sending too many unneccesary i2c messages, only send if servo is not already on
            if(!sfm->servo.isOn)
            {
                // move servo
                pwmWrite(PWM_PIN(sfm->servo.pwmPin), SERVO_ON_PULSE_WIDTH);
                sfm->servo.isOn = true;
            }
            else    // only update position after code has looped at least once
            {
                sfm->servo.position = (sfm->servo.position + SERVO_VELOCITY_LOOP) % COUNTS_PER_REV;
            }
        }
        // check for HE falling edge
        if(sfm->hallEffect.last_value > HE_TRIGGER_VOLTS 
            && sfm->hallEffect.value < HE_TRIGGER_VOLTS)
        {
            sfm->servo.position = sfm->hallEffect.HEPositionOffset;
        }
    }
}

int main()
{
    // TODO: Start GUI Process
    // TODO: Setup code for SPI (Hall Effect Sensors)
    // Setup code for I2C (PWM Controller)
    i2cFD = pca9685Setup(PIN_BASE, PWM_CONTROLLER_ADDR, PWM_FREQ);

    // TODO: Setup code for RGB LED Strip
    
    // TODO: home the module
    while(true)
    {
        Update();
        usleep(LOOP_TIME_US);
    }
}