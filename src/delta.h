#ifndef DELTA_H
#define DELTA_H

#include <Arduino.h>
#include <Servo.h>
#include <math.h>

#include "config.h"
#include "constants.h"

class Delta {
   public:
    Delta() {};

    void begin() {
        _leftServo.attach(LEFT_SERVO_PIN);
        _rightServo.attach(RIGHT_SERVO_PIN);

        _writeServoAngles(0, 0);

        _currentXPos = 0;
        _currentYPos = HOME_Y_POS;

        delay(1000);
    }

    void setSpeed(int speed) { _speed = speed; }

    bool moveTo(float x, float y) {
        float leftShoulderAngle, rightShoulderAngle;
        float startX = _currentXPos;
        float startY = _currentYPos;

        unsigned long lastStepTime = micros();

        if (!_inverseKinematics(x, y, &leftShoulderAngle, &rightShoulderAngle)) return false;

        float distance = sqrt(sq(x - startX) + sq(y - startY));
        if (distance <= 0.0) return true;

        uint16_t steps = distance / INTERPOLATION_STEP_SIZE;
        if (steps < 1) steps = 1;

        unsigned long delayTime = 1000000 / (_speed / INTERPOLATION_STEP_SIZE);

        for (uint16_t i = 0; i < steps - 1; ++i) {
            float progress = static_cast<float>(i + 1) / static_cast<float>(steps);
            float nextX = startX + (x - startX) * progress;
            float nextY = startY + (y - startY) * progress;

            if (!_inverseKinematics(nextX, nextY, &leftShoulderAngle, &rightShoulderAngle)) return false;
            _writeServoAngles(leftShoulderAngle, rightShoulderAngle);

            while (micros() - lastStepTime < delayTime);

            lastStepTime = micros();
        }

        _inverseKinematics(x, y, &leftShoulderAngle, &rightShoulderAngle);
        _writeServoAngles(leftShoulderAngle, rightShoulderAngle);

        _currentXPos = x;
        _currentYPos = y;

        while (micros() - lastStepTime < delayTime);

        return true;
    }

   private:
    Servo _leftServo;
    Servo _rightServo;

    float _currentXPos = 0;
    float _currentYPos = 0;

    int _speed = DEFAULT_SPEED;

    bool _checkWorkspace(float x, float y) {
        float distToLeft = sqrt(sq(x + SHOULDER_SPACING / 2) + sq(y));
        float distToRight = sqrt(sq(x - SHOULDER_SPACING / 2) + sq(y));

        if (distToLeft > UPPER_ARM_LENGTH + LOWER_ARM_LENGTH || distToLeft < abs(UPPER_ARM_LENGTH - LOWER_ARM_LENGTH) ||
            distToRight > UPPER_ARM_LENGTH + LOWER_ARM_LENGTH ||
            distToRight < abs(UPPER_ARM_LENGTH - LOWER_ARM_LENGTH)) {
            return false;
        }
        return true;
    }

    bool _inverseKinematics(float x, float y, float *leftShoulderAngle, float *rightShoulderAngle) {
        float tempLeftAngle, tempRightAngle;

        if (!_checkWorkspace(x, y)) {
            Serial.println("Target is outside workspace");
            return false;
        }

        float distToLeft = sqrt(sq(x + SHOULDER_SPACING / 2) + sq(y));
        float distToRight = sqrt(sq(x - SHOULDER_SPACING / 2) + sq(y));

        float leftElbowAngle = acos((UPPER_ARM_LENGTH_SQUARE + LOWER_ARM_LENGTH_SQUARE - distToLeft * distToLeft) /
                                    (2 * UPPER_ARM_LENGTH * LOWER_ARM_LENGTH));
        float rightElbowAngle = acos((UPPER_ARM_LENGTH_SQUARE + LOWER_ARM_LENGTH_SQUARE - distToRight * distToRight) /
                                     (2 * UPPER_ARM_LENGTH * LOWER_ARM_LENGTH));

        if (leftElbowAngle > MAX_ELBOW_ANGLE_RAD || rightElbowAngle > MAX_ELBOW_ANGLE_RAD) {
            Serial.println("Elbow angle exceeds the limit");
            return false;
        }

        float a1 = acos((UPPER_ARM_LENGTH_SQUARE + distToLeft * distToLeft - LOWER_ARM_LENGTH_SQUARE) /
                        (2 * UPPER_ARM_LENGTH * distToLeft));
        float a2 = atan2(y, (x + SHOULDER_SPACING / 2));

        tempLeftAngle = PI - (a2 + a1);

        float b1 = acos((UPPER_ARM_LENGTH_SQUARE + distToRight * distToRight - LOWER_ARM_LENGTH_SQUARE) /
                        (2 * UPPER_ARM_LENGTH * distToRight));
        float b2 = atan2(y, (x - SHOULDER_SPACING / 2));

        tempRightAngle = (b2 - b1);

        if (tempLeftAngle < MIN_SHOULDER_ANGLE_RAD || tempLeftAngle > MAX_SHOULDER_ANGLE_RAD ||
            tempRightAngle < MIN_SHOULDER_ANGLE_RAD || tempRightAngle > MAX_SHOULDER_ANGLE_RAD) {
            Serial.println("Angles are out of bounds");
            return false;
        }

        *leftShoulderAngle = tempLeftAngle;
        *rightShoulderAngle = tempRightAngle;

        return true;
    }

    void _writeServoAngles(float leftShoulderAngle, float rightShoulderAngle) {
        float leftServoAngleRad = leftShoulderAngle + LEFT_SERVO_ANGLE_OFFSET_RAD;
        float rightServoAngleRad = rightShoulderAngle + RIGHT_SERVO_ANGLE_OFFSET_RAD;

        if (leftServoAngleRad < -SERVO_ANGLE_RANGE_RAD / 2 || leftServoAngleRad > SERVO_ANGLE_RANGE_RAD / 2) {
            Serial.println("Left servo angle is out of range");
            return;
        }

        if (rightServoAngleRad < -SERVO_ANGLE_RANGE_RAD / 2 || rightServoAngleRad > SERVO_ANGLE_RANGE_RAD / 2) {
            Serial.println("Right servo angle is out of range");
            return;
        }

        int16_t leftServoPulseWidth = map(leftServoAngleRad * 100, -SERVO_ANGLE_RANGE_RAD * 50,
                                          SERVO_ANGLE_RANGE_RAD * 50, SERVO_MIN_PULSE_WIDTH, SERVO_MAX_PULSE_WIDTH);

        int16_t rightServoPulseWidth = map(rightServoAngleRad * 100, SERVO_ANGLE_RANGE_RAD * 50,
                                           -SERVO_ANGLE_RANGE_RAD * 50, SERVO_MIN_PULSE_WIDTH, SERVO_MAX_PULSE_WIDTH);

        _leftServo.writeMicroseconds(leftServoPulseWidth);
        _rightServo.writeMicroseconds(rightServoPulseWidth);
    }
};

#endif