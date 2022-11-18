#include "WiringPi/wiringPi/wiringPi.h"
#include "WiringPi/wiringPi/wiringPiI2C.h"
#include <stdio.h>
#include <unistd.h>
#include "pca9685/src/pca9685.h"

#define PWM_CONTROLLER_ADDR 0x40
#define PIN_BASE 300
#define PWM_PIN(x) (PIN_BASE + x)
#define MAX_PWM 4096


/*
 * Calculate the number of ticks the signal should be high for the required amount of time
 */
int calcTicks(float impulseMs, int hertz)
{
	float cycleMs = 1000.0f / hertz;
	return (int)(MAX_PWM * impulseMs / cycleMs + 0.5f);
}

// Continous rotation servos should be ran a 50Hz. They expect a pulse every 20ms.
// Each pulse should be between 1-2ms. Ideally, 1ms is full speed CW, 2ms is full speed CCW, 1.5ms is stopped.
// Servo may need to be calibrated with the set screw on the back.
// Alternetivly, setting pulse width to 0ms will also stop the servo.

int main()
{
    int fd = pca9685Setup(PIN_BASE, PWM_CONTROLLER_ADDR, 50);
    float value = 0;

    wiringPiSetup();

    while(true)
    {
        printf("Enter a number in ms: ");
        scanf("%f", &value);
        pwmWrite(PWM_PIN(8), calcTicks(value, 50));
        usleep(1000);
    }
}