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
#include <cstring>
#include <stdio.h>
#include <unistd.h>
#include <thread>
#include "pca9685/src/pca9685.h"

using std::thread;

// Preprocessor Definitions
#define LOOP_TIME_MS 1
#define LOOP_TIME_US (LOOP_TIME_MS * 1000)

#define NUM_MODULES 16
#define NUM_FLAPS 27

// somewhat arbitrarily set number of increments per revolution to 1000 * NUM_FLAPS. This value can be changed later if needed
#define COUNTS_PER_REV (1000 * NUM_FLAPS)
#define COUNTS_PER_FLAP (COUNTS_PER_REV / NUM_FLAPS)

#define SERVO_VELOCITY_RPM 110.0 // TODO: calibrate May need to calibrate each servo individually
#define SERVO_VELOCITY_LOOP (int)(SERVO_VELOCITY_RPM / 60.0 / 1000.0 * LOOP_TIME_MS * COUNTS_PER_REV) // Counts / loop
#define PWM_FREQ 50 //Hertz (20ms)
#define SERVO_ON_PULSE_WIDTH_MS 1.25f // ms
#define SERVO_OFF_PULSE_WIDTH_MS 0.0f // ms
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
};

// Global variables
int i2cFD = -1;

// volatile because right now assuming this may be modified by another process
char message[NUM_MODULES] = {0};      // 0=Blank. [1-26]=[A-Z]
bool newMessageReceived = false;

// TODO: ADC_Read
float ADC_Read(unsigned int adcID, unsigned int adcPin)
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
        if(sfm->servo.position >= sfm->servo.targetPosition && sfm->servo.position < sfm->servo.targetPosition + (COUNTS_PER_FLAP / 4))
        {
            // stop servo
            // to avoid sending too many unneccesary i2c messages, only send if servo is not already off
            if(sfm->servo.isOn)
            {
                printf("stopping servo %d\n", i);
                pwmWrite(PWM_PIN(sfm->servo.pwmPin), SERVO_OFF_PULSE_WIDTH);
                sfm->servo.isOn = false;
            }
        }
        else
        {
            // to avoid sending too many unneccesary i2c messages, only send if servo is not already on
            if(!sfm->servo.isOn)
            {
                // move servo
                printf("moving servo %d\n", i);
                pwmWrite(PWM_PIN(sfm->servo.pwmPin), SERVO_ON_PULSE_WIDTH);
                sfm->servo.isOn = true;
            }
            else    // only update position after code has looped at least once
            {
                sfm->servo.position = (sfm->servo.position + SERVO_VELOCITY_LOOP) % COUNTS_PER_REV;
                if(i==8) printf("Servo %d position: %d\n%d\n", i, sfm->servo.position, SERVO_VELOCITY_LOOP);
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

void console_input()
{
    char new_message[NUM_MODULES] = {0};
    while(true)
    {
        // get new message
        printf("Enter String: ");
        fgets(new_message, NUM_MODULES, stdin);

        // check for invalid characters and convert chars to [0-26] format
        // fill in an extra characters with space
        for(int i = 0; i < NUM_MODULES; ++i)
        {
            if(new_message[i] == '\n' || new_message[i] == '\0')
            {
                memset(&new_message[i], 0, NUM_MODULES - i);
                break;
            }
            else if(new_message[i] >= 'A' && new_message[i] <= 'Z')
            {
                new_message[i] -= 64;
            }
            else if(new_message[i] >= 'a' && new_message[i] <= 'z')
            {
                new_message[i] -= 96;
            }
            else
            {
                // either a proper space, or an invalid character, set to space for both
                new_message[i] = 0;
            }
        }

        // do not set new message until previous message has been processed
        while(newMessageReceived)
        {
            usleep(LOOP_TIME_US);
        }
        
        memcpy(message, new_message, NUM_MODULES);
        newMessageReceived = true;
    }
}

int main()
{
    wiringPiSetup();
    
    // Start user input thread
    thread userInputThread(console_input);
    // TODO: Setup code for SPI (Hall Effect Sensors)
    // Setup code for I2C (PWM Controller)
    // this line creates 16 virtual pins (starting at PIN_BASE) that can passed to
    // WiringPi's pwmWrite functiom
    i2cFD = pca9685Setup(PIN_BASE, PWM_CONTROLLER_ADDR, PWM_FREQ);

    // TODO: Setup code for RGB LED Strip
    
    // TODO: home the module
    while(true)
    {
        Update();
        usleep(LOOP_TIME_US);
    }

    userInputThread.join();
}