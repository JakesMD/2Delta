![2Delta](images/2delta.gif)

# 2Delta
**An open-source 2-axis delta robot.**

The great thing about delta, is it only uses 2 small servo motors, but it has a quite a large workspace.


## Usage

### Configure [config.h](src/config.h)
Carefully adjust the values based on your robot's physical dimensions and the specifications in your servo motor datasheets.

**Incorrect configuration will lead to self-destruction!**

### Basic control

```c++
#include <delta.h>
#include <Servo.h>

Delta delta;

void setup() {
    Serial.begin(9600);

    // CAREFUL: This will move the servos to the home position
    //          with no control over speed!!
    delta.begin();
}

void loop() {
    delta.setSpeed(50); // Speed in mm/s

    // Move to (x, y) in mm.
    //
    // This will return `false` if
    // - the position is out of bounds
    // - getting there would cause a collision.
    delta.moveTo(100, 200); 
    delta.setSpeed(200);
    delta.moveTo(-50, 300);
}
```

`moveTo` is blocking, so let me know if you need a non-blocking version.

### Uploading the code
**Turn off the power to the servos before uploading!**

Uploading can interfere with the PWM signals and cause the servos to go crazy until the robot is destroyed.

In future versions, I'll add an upload button, which will detach the servos before uploading.