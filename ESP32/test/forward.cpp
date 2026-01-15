#include "esp32-hal.h"
#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>
#include <math.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN 100
#define SERVOMAX 600
#define FREQUENCY 60

struct legIndex {
    uint8_t knee;
    uint8_t hip;
};

legIndex legs[4] = {{0, 3}, {4, 7}, {8, 11}, {12, 15}};

uint16_t pulseWidth(int angle) {
    angle = constrain(angle, 0, 180);
    float unit_angle = (SERVOMAX - SERVOMIN) / 180.0f;
    return (uint16_t)(angle * unit_angle + SERVOMIN);
}

void moveTwoLegs(legIndex legA, int hA_Start, int hA_End, int kA_Start, int kA_End, legIndex legB, int hB_Start, int hB_End, int kB_Start, int kB_End, int stepDelayMs) {
    int hA_Delta = hA_End - hA_Start;
    int kA_Delta = kA_End - kA_Start;
    int hB_Delta = hB_End - hB_Start;
    int kB_Delta = kB_End - kB_Start;

    int steps = max(abs(hA_Delta), abs(kA_Delta));
    steps = max(steps, abs(hB_Delta));
    steps = max(steps, abs(kB_Delta));

    if (steps == 0)
        return;

    for (int i = 0; i <= steps; i++) {
        float progress = (float)i / steps;

        int current_hA = hA_Start + (hA_Delta * progress);
        int current_kA = kA_Start + (kA_Delta * progress);

        int current_hB = hB_Start + (hB_Delta * progress);
        int current_kB = kB_Start + (kB_Delta * progress);

        pwm.setPWM(legA.hip, 0, pulseWidth(current_hA));
        pwm.setPWM(legA.knee, 0, pulseWidth(current_kA));

        pwm.setPWM(legB.hip, 0, pulseWidth(current_hB));
        pwm.setPWM(legB.knee, 0, pulseWidth(current_kB));

        delay(stepDelayMs);
    }
}


void moveFourLegs(
    legIndex legA, int hA_Start, int hA_End, int kA_Start, int kA_End,
    legIndex legB, int hB_Start, int hB_End, int kB_Start, int kB_End,
    legIndex legC, int hC_Start, int hC_End, int kC_Start, int kC_End,
    legIndex legD, int hD_Start, int hD_End, int kD_Start, int kD_End,
    int stepDelayMs
) {
    int hA_D = hA_End - hA_Start;
    int kA_D = kA_End - kA_Start;
    int hB_D = hB_End - hB_Start;
    int kB_D = kB_End - kB_Start;
    int hC_D = hC_End - hC_Start;
    int kC_D = kC_End - kC_Start;
    int hD_D = hD_End - hD_Start;
    int kD_D = kD_End - kD_Start;

    int steps = 0;
    steps = max(steps, abs(hA_D));
    steps = max(steps, abs(kA_D));
    steps = max(steps, abs(hB_D));
    steps = max(steps, abs(kB_D));
    steps = max(steps, abs(hC_D));
    steps = max(steps, abs(kC_D));
    steps = max(steps, abs(hD_D));
    steps = max(steps, abs(kD_D));

    if (steps == 0) return;

    for (int i = 0; i <= steps; i++) {
        float p = (float)i / steps;

        pwm.setPWM(legA.hip,  0, pulseWidth(hA_Start + hA_D * p));
        pwm.setPWM(legA.knee, 0, pulseWidth(kA_Start + kA_D * p));

        pwm.setPWM(legB.hip,  0, pulseWidth(hB_Start + hB_D * p));
        pwm.setPWM(legB.knee, 0, pulseWidth(kB_Start + kB_D * p));

        pwm.setPWM(legC.hip,  0, pulseWidth(hC_Start + hC_D * p));
        pwm.setPWM(legC.knee, 0, pulseWidth(kC_Start + kC_D * p));

        pwm.setPWM(legD.hip,  0, pulseWidth(hD_Start + hD_D * p));
        pwm.setPWM(legD.knee, 0, pulseWidth(kD_Start + kD_D * p));

        delay(stepDelayMs);
    }
} 


void setup() {
    Serial.begin(115200);
    pwm.begin();
    pwm.setPWMFreq(FREQUENCY);

    for (int i = 0; i <= 3; i++) {
        pwm.setPWM(legs[i].knee, 0, pulseWidth(180));
        pwm.setPWM(legs[i].hip, 0, pulseWidth(90));
    }
    delay(1000);
    // moveFourLegs(legs[0], 90, 90, 180, 150 , legs[1], 90, 50, 180, 160, legs[2], 90, 130 , 180 , 150 , legs[3], 90, 150 , 180 , 160, 1);
    // moveFourLegs(legs[0], 90, 60, 180, 160 , legs[1], 90, 125, 180, 180, legs[2], 90, 90, 180 , 160 , legs[3], 90, 60, 180 , 180, 1);
    // moveFourLegs(legs[0], 60, 110, 160, 180 , legs[1], 125, 90, 180, 160, legs[2], 90, 60, 160 , 180 , legs[3], 60, 130, 180 , 160, 1);
}

void loop() {
    moveFourLegs(legs[0], 90, 60, 180, 140 , legs[1], 90, 125, 180, 180, legs[2], 90, 90, 180 , 140 , legs[3], 90, 60, 180 , 180, 1);
    delay(100)
    moveFourLegs(legs[0], 60, 110, 140, 180 , legs[1], 125, 90, 180, 140, legs[2], 90, 60, 140 , 180 , legs[3], 60, 130, 180 , 140, 1);
}
