# Autonomous Guided Vehicles -- AGV Experiments

## AGV Experiment [1] Discovery
In this experiment, we will be demonstrating how a AGV works, using 'Makeblock Ranger' and associated Arduino Libraries. Requirement for this experiment is to program 'Ranger' to navigate a pre-defined route using and black tape, grey-scale sensors, ultrasonic sensor and motor control. Experiment must demonstrate health and safety, safety features, to ensure no collisions or rogue activity.  

Although not functional in this experiment, we will be gathering stateful metrics about the system that will aid in our next experiment where we will look at error detection and correction, using Newtons Laws of Motion. 

**State Measurements:**
- [] Grey-scale Sensor (GS)
- [] Gyroscopic Axis (axis)
- [] Ultrasonic Sensor (SW)
- [] Light Encoded Motors (LE Motor)

First, let's start by setting up your development environment.

Open Terminal and navigate to your Arduino directory, usually `~/user/documents/arduino`.

```sh
cd ~/user/documents/arduino
```

Next we need to clone the Arduino code for this experiment:

```sh
git clone https://github.com/RAZA7909/Polaris-eFoundry-AGV-Repository
```
```sh
cd ~/user/documents/arduino/Polaris_eFoundry_AGV_Repository
```
```sh
ls -lst
```

You should see 1 files and 1 directory listed:

- [] Polaris_eFoundry_AGV_Repository
- [] README.md

navigate to the Polaris_eFoundry_AGV_Repository directory
```sh
cd Polaris_eFoundry_AGV_Repository
```
You should see 3 files listed:

- [] Polaris_eFoundry_AGV_Repository.ino
- [] Polaris_eFoundry_AGV_Repository.h
- [] README.md
- 
Now we need to follow the Makeblock tutorial on setting up Ranger for Arduino:
[Makeblock - Arduino tutorial:](https://support.makeblock.com/hc/en-us/articles/1500004053721-Programming-mBot-Ranger-in-Arduino "Programming mBot Ranger in Arduino")

A good coding reference for the makeblock libraries can be found [here.](https://docs.google.com/document/d/16uXDUmgN_9jM2sp_KGJtZZfQTpQ2-PzLDtjUFla_FcA/edit?tab=t.0#heading=h.7sudpr9ypkx "Makeblock Ranger Arduino Coding Reference") 

### Code Explaination
Open the `AV_FollowLine_ObsAvoid_AudioAlert.ino` file with Arduino. []
```cpp
#include <MeAuriga.h>
#include "AV_FollowLine_ObsAvoid_AudiAlert.h"

void setup() {
  Serial.begin(115200);
  buzzer.setpin(BUZZER_PORT);
  buzzer.tone(600, 1000);
  // Set PWM 8KHz
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);
  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);
  // Ensure motors are stopped at startup
  Encoder_1.setMotorPwm(0);
  Encoder_2.setMotorPwm(0);
}

void loop() {
  int sensorState = lineFinder.readSensors();
  if (millis() - LAST_BUZZER >= BUZZER_INTERVAL) {
      LAST_BUZZER = millis();
      if (BUZZER_ON) {
        buzzer.noTone();  // Turn off buzzer
      } else {
        buzzer.tone(600, 150);  // Sound buzzer at 1kHz
      }
      BUZZER_ON = !BUZZER_ON;
    }
  if (sensorState == S1_OUT_S2_OUT && LAST_TURN != NULL) {
    TurnRight();
  } else {
    Drive();
  }
}
```

As we are using Ranger, we need calling in the the board specific library MeAuriga.h which is in the header file which we will discuss later. This holds many accessible classes and functions, which Makeblock have kindly created to simplify and enable linear scripting for junior engineers. 

In further experiments, we will be looking at .cpp abstraction using Makeblock as a prime example of good implementation of coding standards with abstraction layers, but for now, lets understand what we are accessing within our use case.

First we create a serial on the 115200Mkz range for logging data.
```cpp
Serial.begin(115200);
```
*It is important to note*, logging to serial eats up system memory resource which may lead to system failure. Good practice would be to periodically post to *db* and flush serial on successful response from messaging proxy. 

Next we initialize the buzzer object and set its pins and test tone. We have abstracted alot of aspects of the buzzer object to our header file, which will be covered further in the experiment.
```cpp
buzzer.setpin(BUZZER_PORT);
buzzer.tone(600, 1000);
```
The next thing we need to do is set our timers for pulse wave monitor (PWM). As the Auriga board is an aTmega32U, we can access 4 onboard timers, but for our experiment, we only need 2.

**PWM Timers**
Here we are setting the PWM to fast mode at the 8Mkz range. Defining the wave range to use can be tricky, but as a rule of thumb, it can be defined by the following equation `PWMRange = CPUClock/(prescaler * 256)` in our instance we know our board operates at 16000Mhz, and through our resource budget and allocation, we know we have 8 bits, which can be allocated to PWM prescaler. Our equation now looks like this `PWMRange = 16000/(8*256)` which equates to `PWMRange = 7.8125` which means our PWM operates within the 8Mhz range.

Now we know what range we need to operate, we can go ahead and assign the relevant values 
```cpp
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);
  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);
```
`TCCR1A`, `TCCR1B`, `TCCR2A` and `TCCR2B` are control registers for each motors PWM. `_BV()` is a macro that creates a bitmask for setting specific bits, within this macro we can set wave generation mode (WGM) and clock speed (CS). In our scenario, we set Fast PWM Mode by assigning `WGM10` to `TCCR1A`, `WGM12` to `TCCR1B`, `WGM21` and `WGM20` to `TCCR2B`. Prescaler clock speed is defined by passing `CS11` to `TCCR1B` and `CS21` to `TCCR2B`.

**Main Loop Explanation**
```cpp
void loop() {
  int sensorState = lineFinder.readSensors();
  if (millis() - LAST_BUZZER >= BUZZER_INTERVAL) {
      LAST_BUZZER = millis();
      if (BUZZER_ON) {
        buzzer.noTone();  // Turn off buzzer
      } else {
        buzzer.tone(600, 150);  // Sound buzzer at 1kHz
      }
      BUZZER_ON = !BUZZER_ON;
    }
  Drive();
}
```
Within the above loop, we are first initializing the line finding sensorState object. This object will help with further safety checks. Next we create a buzzer to buzz every second during operation, once that's done we drive.

Now your probably thinking to yourself, where is the magic and logic making the AGV `Drive();`. If you remember I said Makeblock are a prime example of good abstraction and coding standards, well we followed suit, and abstracted the majority of our code out into a `AV_FollowLine_ObsAvoid_AudiAlert.h` header file.

**Abstraction Layer Using Header Files**
Looking at the code below, you can see the `#ifndef`, this is a safety clause which basically says, if this file is called and this `AV_FollowLine_ObsAvoid_AudiAler_h` object not defined, do something. In our case we want to define what to be included in this system. This clause is closed with `#endif`.

In this file we include the `MeAuriga.h` and `Arduino.h` libraries. We also define a number of variables, set ports and addresses.

```cpp
#ifndef AV_FollowLine_ObsAvoid_AudiAler_h
#define AV_FollowLine_ObsAvoid_AudiAler_h
#include <MeAuriga.h>
#include "Arduino.h"
#define BUZZER_PORT 45
#define SAFE_DISTANCE 20

MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);
MeLineFollower lineFinder(PORT_9);
MeUltrasonicSensor ultrasonic(PORT_10);
MeBuzzer buzzer;
MeGyro gyro_ext(0, 0x68);  //external gryo sensor
MeGyro gyro(1, 0x69);

int16_t MOVE_SPEED = 65;  // 50% speed
int16_t MAX_SPEED = 100;
unsigned long LAST_BUZZER = 0;              // Timer for buzzer
const unsigned long BUZZER_INTERVAL = 500;  // 1 second interval
bool BUZZER_ON = false;
char LAST_TURN;
```
As mentioned previously, stateful metrics are very important to understand errors and methods of correction. Although not used in this experiment, we have create a robotState class, with getter and setter constructors, enabling real time analysis, depending on hardware resource budget. These metric can be particularly useful when calculating force of motion and inertia, to enable precision navigation and motor control.
```cpp
class robotState {
  private:
    int _xAxis;
    int _yAxis;
    int _zAxis;
    char _lastTurn;
    int _objectDistance;
  public:

  void getState() {
    _xAxis = gyro.getAngleX();
    _yAxis = gyro.getAngleY();
    _zAxis = gyro.getAngleZ();
    _lastTurn = LAST_TURN;
    _objectDistance = ultrasonic.distanceCm();
  }

  int get_xAxis() {
    return _xAxis;
  }
  int get_yAxis() {
    return _yAxis;
  }
  int get_zAxis() {
    return _zAxis;
  }
  char get_lastTurn() {
    return _lastTurn;
  }
  int getObjectDistance() {
    return _objectDistance;
  }
} ROBOT_STATE;
```
As mentioned, careful abstraction, makes programming on the interface a whole lot easier, when functions and sub routines are moved away from main loop, it also makes main a lot easier to read. As you can see below, we have created functions for each motion needed within our AGV experiment.
```cpp
void Forward() {
  if (ultrasonic.distanceCm() > SAFE_DISTANCE) {
    Encoder_1.setMotorPwm(-MOVE_SPEED);
    Encoder_2.setMotorPwm(MOVE_SPEED);
  } else {
    Stop();
  }
}
```
The negative speed (-moveSpeed) for Encoder_1 and positive speed (moveSpeed) for Encoder_2 would ensure that both motors turn in ways that complement each other to move the robot forward. This is a common scenario in differential drive robots, where two wheels are independently driven and the direction of each wheel needs to be controlled separately to achieve the desired direction of the overall robot movement.

Now we have explained how and why we have moved these functions out, lets explain the magic and logic behind the `Drive();` function. Within the Auriga library, there is a class to define grey-scale sensor states, which returns on of the following states based on what the sensor picks up: 

- S1_IN_S2_IN = Both sensors pick up line
- S1_IN_S2_OUT = Left Sensor senses line
- S1_OUT_S2_IN = Right Sensor senses line
- S1_OUT_S2_OUT = No line detected.

In this experiment, we created a switch case as it is more efficient than conditional if. If both sensors detect line, we tell it to drive forward by calling the function created previously `Forward();`. We also use similar logic for each turn and stop. In addition we have a safety stop built in with an alarm, which will inform people in the vicinity of the danger.
```cpp
void Drive() {
  int sensorState = lineFinder.readSensors();
  switch (sensorState) {
    case S1_IN_S2_IN:
      Forward();
      LAST_TURN = 'F';
      break;
    case S1_IN_S2_OUT:
      TurnLeft();  // Only left sensor detects the line, turn left
      LAST_TURN = 'L';
      break;
    case S1_OUT_S2_IN:
      TurnRight();  // Only right sensor detects the line, turn right
      LAST_TURN = 'R';
      break;
    case S1_OUT_S2_OUT:
      Stop();
      break;
  }

  if (ultrasonic.distanceCm() < SAFE_DISTANCE) {
    Stop();
    buzzer.tone(1000, 1000);
    buzzer.tone(600, 1000);
  } 
}
```
Well there you have it, we have now conducted our AGV discovery using Makeblock Ranger as our eductaional tool. 
Join us next time as we look to incorperate Newtons Laws of Motion and the Force of Inertia, to understand how precision navigation is conducted

## AGV Experiment [2] Intro - State Processing using Newtons Laws of Motion
AGVs are required to be stateful by nature. All systems use the following metrics, among others, to define state and actions to take, to correct and complete their function:

**State Measurements:**
- [] Grey-scale Sensor (GS)
- [] Gyroscopic Axis (axis)
- [] Ultrasonic Sensor (SW)
- [] Light Encoded Motors (LE Motor)

Through scientific use of Newton's laws of physics, sensor fusion, kinematics, and dynamics, coupled with PID correction, an AGV can navigate with relative precision, with built-in stop state and emergency cut-off.

***Stay tuned for our next experiment!!!***

## References

- [Learn more about Law of Inertia:](https://stileapp.com/static/CLL%20handouts/Lesson_048_handout.pdf "Law of Inertia: Hands-free driving")
- [Learn more about Newtons Laws of Motion:](https://texasgateway.org/resource/newtons-three-laws-motion.com "Newton's Three Laws of Motion")
- [Science Buddies - Learn more about Newtons Laws of Motion:](https://www.sciencebuddies.org/blog/newton-laws-science-lessons.com "Science Buddies - About Newton's Laws of Motion")
- [Stanford University's EE259: Principles of Sensing for Autonomy:](https://ee259.stanford.edu.com "Stanford University's EE259: Principles of Sensing for Autonomy")
- [Makeblock - Arduino tutorial:](https://support.makeblock.com/hc/en-us/articles/1500004053721-Programming-mBot-Ranger-in-Arduino "Programming mBot Ranger in Arduino")
- [Arduino Coding Reference](https://docs.google.com/document/d/16uXDUmgN_9jM2sp_KGJtZZfQTpQ2-PzLDtjUFla_FcA/edit?tab=t.0#heading=h.7sudpr9ypkx "Makeblock Ranger Arduino Coding Reference")
- [Polaris_eFoundry_AGV_Repository](https://github.com/RAZA7909/Polaris_eFoundry_AGV_Repository)
