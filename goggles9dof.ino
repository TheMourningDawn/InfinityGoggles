// Googly Eye Goggles
// By Bill Earl
// For Adafruit Industries
//
// The googly eye effect is based on a physical model of a pendulum.
// The pendulum motion is driven by accelerations in 2 axis.
// Eye color varies with orientation of the magnetometer
#include <Wire.h>
#include <SPI.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h>
#include <Adafruit_Simple_AHRS.h>
#include <FastLED.h>


#define DATAPIN_LEFT    6
#define CLOCKPIN_LEFT   9
#define DATAPIN_RIGHT    0
#define CLOCKPIN_RIGHT   1
#define NUM_LEDS 21 //Number of LEDS per lense

CRGB leftLense[NUM_LEDS];
CRGB rightLense[NUM_LEDS];

Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0();
sensors_event_t accel, mag, gyro, temp;
Adafruit_Simple_AHRS ahrs(&lsm.getAccel(), &lsm.getMag());
 
float pos = 6;  // Starting center position of pupil
float increment = 2 * 3.14159 / 21; // distance between pixels in radians
float MomentumH = 0; // horizontal component of pupil rotational inertia
float MomentumV = 0; // vertical component of pupil rotational inertia

// Tuning constants. (a.k.a. "Fudge Factors)  
// These can be tweaked to adjust the liveliness and sensitivity of the eyes.
const float friction = 0.999; // frictional damping constant.  1.0 is no friction.
const float swing = 60;  // arbitrary divisor for gravitational force
const float gravity = 200;  // arbitrary divisor for lateral acceleration
const float nod = 7.5; // accelerometer threshold for toggling modes

long nodStart = 0;
long nodTime = 2000;

bool antiGravity = true;  // The pendulum will anti-gravitate to the top.
bool mirroredEyes = false; // The left eye will mirror the right.

const float halfWidth = 2.0; // half-width of pupil (in pixels)

// Pi for calculations - not the raspberry type :: 1.25
const float Pi = 3.14159;

void setupSensor() {
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_6G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_16G);
  
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_12GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);
//  Serial.println("did this?");
}

void setup(void) {
   Serial.begin(9600);
  FastLED.addLeds<APA102, DATAPIN_LEFT, CLOCKPIN_LEFT>(leftLense, NUM_LEDS);
  FastLED.addLeds<APA102, DATAPIN_RIGHT, CLOCKPIN_RIGHT>(rightLense, NUM_LEDS);

   // Try to initialise and warn if we couldn't detect the chip
  if (!lsm.begin()) {
//    Serial.println("Oops ... unable to initialize the LSM9DS0. Check your wiring!");
    while (1);
  }
  setupSensor();
  resetModes();
}
  
// main processing loop
void loop(void) 
{
   // Read the magnetometer and determine the compass heading:
   lsm.getEvent(&accel, &mag, &gyro, &temp); 
   
   sensors_vec_t   orientation;
   ahrs.getOrientation(&orientation);
   
   // Calculate the angle of the vector z,x from magnetic North
//   float heading = atan2(mag.magnetic.x,mag.magnetic.z) * (180 / Pi);
    float heading = orientation.heading;

   // Normalize to 0-360 for a compass heading
   if (heading < 0) {
      heading = 360 + heading;
   }

   // Check for mode change commands
//   CheckForNods(lsm);

   // apply a little frictional damping to keep things in control and prevent perpetual motion
   MomentumH *= friction;
   MomentumV *= friction;

   // Calculate the horizontal and vertical effect on the virtual pendulum
   // 'pos' is a pixel address, so we multiply by 'increment' to get radians.
   float TorqueH = cos(pos * increment);  // peaks at top and bottom of the swing
   float TorqueV = sin(pos * increment);    // peaks when the pendulum is horizontal

   // Add the incremental acceleration to the existing momentum
   // This code assumes that the accelerometer is mounted upside-down, level
   // and with the X-axis pointed forward.  So the Y axis reads the horizontal
   // acceleration and the inverse of the Z axis is gravity.
   // For other orientations of the sensor, just change the axis to match.
   MomentumH += TorqueH * accel.acceleration.z / swing;
   if (antiGravity) {
     MomentumV += TorqueV * -accel.acceleration.y / gravity;
   } else {
     MomentumV -= TorqueV * -accel.acceleration.y / gravity;
   }

   // Calculate the new position
   pos += MomentumH + MomentumV;
   
   // handle the wrap-arounds at the top
   while (round(pos) < 0) pos += 21.0;
   while (round(pos) > 20) pos -= 21.0;

   // Now re-compute the display
   for (int i = 0; i <21; i++) {
      // Compute the distance bewteen the pixel and the center
      // point of the virtual pendulum.
      float diff = i - pos;

      // Light up nearby pixels proportional to their proximity to 'pos'
      if (fabs(diff) <= halfWidth) {
         CRGB color;
         float proximity = halfWidth - fabs(diff) * 200; //Not being used right now

         // pick a color based on heading & proximity to 'pos'
         color = selectColor(heading);
         
         // do both eyes
         leftLense[i] = color;
         float rightLenseOffset = i - 10;
         while (round(rightLenseOffset) < 0) rightLenseOffset += 21.0;
         while (round(rightLenseOffset) > 20) rightLenseOffset -= 21.0;
         int rightLenseIndex = fabs(rightLenseOffset);
         if (mirroredEyes) {
           rightLense[rightLenseIndex] = color;
         } else {
           rightLense[rightLenseIndex] = color;
         }
      } else {// all others are off
         float rightLenseOffset = i - 10;
         while (round(rightLenseOffset) < 0) rightLenseOffset += 21.0;
         while (round(rightLenseOffset) > 20) rightLenseOffset -= 21.0;
         int rightLenseIndex = fabs(rightLenseOffset);
         leftLense[i] = CRGB(0,0,0);
         if (mirroredEyes) {
           rightLense[rightLenseIndex] = CRGB(0,0,0);
         } else {
           rightLense[rightLenseIndex] = CRGB(0,0,0);
         }
      }
   }
   FastLED.show();
}

// choose a color based on the compass heading and proximity to "pos".
CRGB selectColor(float heading) {
     int convertedHeading = map(heading, 0, 360, 0, 255);
     return CHSV(convertedHeading, 255, 180);
}

// monitor orientation for mode-change 'gestures'
void CheckForNods(sensors_event_t event){
   if (accel.acceleration.x > nod) {
     if (millis() - nodStart > nodTime) {
       antiGravity = false;  
       nodStart = millis(); // reset timer     
       spinDown();
     }
   } else if (accel.acceleration.x < -(nod + 1)) {
     if (millis() - nodStart > nodTime) {
       antiGravity = true;  
       spinUp();
       nodStart = millis(); // reset timer     
     }
   } else if (accel.acceleration.y > nod) {
     if (millis() - nodStart > nodTime) {
       mirroredEyes = false;  
       spinDown();
       nodStart = millis(); // reset timer     
     }
   } else if (accel.acceleration.y < -nod) {
     if (millis() - nodStart > nodTime) {
       mirroredEyes = true;  
       spinUp();
       nodStart = millis(); // reset timer     
      }
   } else { // no nods in progress
     nodStart = millis(); // reset timer
   }
}

// Reset to default
void resetModes() {
   antiGravity = false;
   mirroredEyes = true;
   
   /// spin-up
   spin(CRGB(255, 0, 0), 1, 500);
   spin(CRGB(0, 255, 0), 1, 500);
   spin(CRGB(0, 0, 255), 1, 500);
   spinUp();
}

// gradual spin up
void spinUp() {
   for (int i = 300; i > 0;  i -= 20) {
     spin(CRGB::White, 1, i);
   }
   pos = 0;
   // leave it with some momentum and let it 'coast' to a stop
   MomentumH = 3;  
}

// Gradual spin down
void spinDown() {
   for (int i = 1; i < 300; i++) {
     spin(CRGB::White, 1, i += 20);
   }
   // Stop it dead at the top and let it swing to the bottom on its own
   pos = 0;
   MomentumH = MomentumV = 0;
}


// utility function for feedback on mode changes.
void spin(CRGB color, int count, int time) {
  for (int j = 0; j < count; j++) {
    for (int i = 0; i < 21; i++) {
      rightLense[i] = color;
      leftLense[i] = color;
      FastLED.show();
      delay(max(time / 21, 1));
      leftLense[i] = CRGB::Black;
      rightLense[i] = CRGB::Black;
      FastLED.show();
    }
  }
}

