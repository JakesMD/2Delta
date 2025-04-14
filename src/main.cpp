#include <Arduino.h>
#include <Servo.h>
#include <math.h>

#include "delta.h"

Delta delta;

uint16_t pressY = 277;
uint16_t releaseY = 250;

unsigned long millisPlayed;

byte bpm = 45;
uint16_t msPerBeat = 60000 / bpm;

void playNote(int note, float noteValue) {
    delta.moveToPos(note, releaseY);

    while (millis() < millisPlayed);

    delta.moveToPos(note, pressY);
    int delayTime = msPerBeat * noteValue * 0.25;
    if (delayTime > 0) delay(delayTime);
    delta.moveToPos(note, releaseY);

    millisPlayed += (msPerBeat * (noteValue * 100)) / 100;
}

void leftHand() {
    int b = 25 * 4;
    int c = 25 * 3;
    int d = 25 * 2;
    int e = 25;
    int f = 0;
    int g = -25;
    int a = -25 * 2;
    int b2 = -25 * 3;
    int c2 = -25 * 4;

    millisPlayed = millis();
    delta.moveToPos(e, releaseY);
    while (millis() - millisPlayed < 2000);

    millisPlayed = millis();

    playNote(e, 1);
    playNote(c, 1);

    playNote(g, 2);
    playNote(b, 2);

    playNote(c, 2);
    playNote(e, 1);
    playNote(c2, 1);

    playNote(d, 1);
    playNote(b2, 1);
    playNote(c2, 1);
    playNote(c, 1);

    playNote(g, 2);
    playNote(e, 1);
    playNote(c, 1);

    playNote(g, 2);
    playNote(b, 2);

    playNote(c, 2);
    playNote(a, 1);
    playNote(c2, 1);

    playNote(f, 1);
    playNote(c2, 1);
    playNote(g, 1);
    playNote(b2, 1);

    playNote(c2, 2);
}

void rightHand() {
    int g = 25 * 3;
    int c = 0;
    int d = -25;
    int e = -25 * 2;
    int f = -25 * 3;
    int g2 = -25 * 4;

    millisPlayed = millis();
    delta.moveToPos(c, releaseY);
    while (millis() - millisPlayed < 2000);

    millisPlayed = millis();

    playNote(c, 1);
    playNote(e, 1);

    playNote(d, 1);
    playNote(g, 1);
    playNote(d, 0.5);
    playNote(e, 0.5);
    playNote(f, 0.5);
    playNote(d, 0.5);

    playNote(e, 1);
    playNote(c, 1);
    playNote(g2, 2);

    playNote(f, 2);
    playNote(e, 0.5);
    playNote(d, 0.5);
    playNote(e, 0.5);
    playNote(f, 0.5);

    playNote(e, 1);
    playNote(d, 1);
    playNote(c, 1);
    playNote(e, 1);

    playNote(d, 1);
    playNote(g, 1);
    playNote(d, 0.5);
    playNote(e, 0.5);
    playNote(f, 0.5);
    playNote(d, 0.5);

    playNote(e, 1);
    playNote(c, 1);
    playNote(f, 2);

    playNote(e, 2);
    playNote(d, 0.5);
    playNote(c, 0.5);
    playNote(d, 0.5);
    playNote(e, 0.5);

    playNote(c, 2);
}

void setup() {
    Serial.begin(9600);

    delta.begin();
    delta.setSpeed(200);

    delay(4000);
}

void loop() {
    rightHand();
    delay(1000);
}