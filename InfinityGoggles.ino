// Googly Eye Goggles
// By Bill Earl
// For Adafruit Industries
// Edited by
// Charlie Hendricks
// because I want what I want?
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


#define DATAPIN_LEFT    0
#define CLOCKPIN_LEFT   1
#define DATAPIN_RIGHT    6
#define CLOCKPIN_RIGHT   9
#define NUM_LEDS 21 //Number of LEDS per lense

CRGB leftLense[NUM_LEDS];
CRGB rightLense[NUM_LEDS];

Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0();
sensors_event_t accel, mag, gyro, temp;

float pos = 6;  // Starting center position of pupil
float increment = 2 * 3.14159 / NUM_LEDS; // distance between pixels in radians
float MomentumH = 0; // horizontal component of pupil rotational inertia
float MomentumV = 0; // vertical component of pupil rotational inertia

// Tuning constants. (a.k.a. "Fudge Factors)
// These can be tweaked to adjust the liveliness and sensitivity of the eyes.
const float friction = 0.985; // frictional damping constant.  1.0 is no friction.
const float swing = 20;  // arbitrary divisor for gravitational force
const float gravity = 100;  // arbitrary divisor for lateral acceleration
const float gestureThreshold = -0.80; // accelerometer threshold for toggling modes

long gestureStart = 0;
long gestureHoldTime = 3000;

int modeNumber = 0;

bool pendulum = true;
bool antiGravity = true;  // The pendulum will anti-gravitate to the top.
bool mirroredEyes = false; // The left eye will mirror the right.

const float pupilRadius = 2; // half-width of pupil (in pixels)

// Pi for calculations - not the raspberry type
const float Pi = 3.14159;

void setupSensor()
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G); // 2G, 4G, 6G, 8G, 16G

  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS); //2GAUSS,4GAUSS,8GAUSS,12GAUSS

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS); // 245DPS,500DPS,2000DPS
}

void setup(void)
{
  FastLED.addLeds<APA102, DATAPIN_LEFT, CLOCKPIN_LEFT>(leftLense, NUM_LEDS);
  FastLED.addLeds<APA102, DATAPIN_RIGHT, CLOCKPIN_RIGHT>(rightLense, NUM_LEDS);

  // Try to initialise
  if (!lsm.begin())
  {
    while (1);
  }
  setupSensor();
  resetModes();
}

int test = 0;
// main processing loop
void loop(void)
{
  // Read the magnetometer and determine the compass heading:
  lsm.getEvent(&accel, &mag, &gyro, &temp);
  // Check for mode change commands
  checkForGestures(accel);

  CRGB magHeadingBasedColor = convertHeadingToColor(fabs(round(mag.magnetic.z * 100)));

  if(pendulum == true)
  {
    pendulumMode(magHeadingBasedColor);
  }
  else 
  {
    staticMode(magHeadingBasedColor);
  }

}

void staticMode(CRGB color)
{
  fill_solid(&(rightLense[0]), NUM_LEDS, color);
  fill_solid(&(leftLense[0]), NUM_LEDS, color);
  FastLED.show();
}

void pendulumMode(CRGB color)
{
  // apply a little frictional damping to keep things in control and prevent perpetual motion
  MomentumH *= friction;
  MomentumV *= friction;

  // Calculate the horizontal and vertical effect on the virtual pendulum
  // 'pos' is a pixel address, so we multiply by 'increment' to get radians.
  float TorqueH = cos(pos * increment);  // peaks at top and bottom of the swing
  float TorqueV = sin(pos  * increment);   // peaks when the pendulum is horizontal

  // Add the incremental acceleration to the existing momentum
  // This code assumes that the accelerometer is mounted upside-down, level
  // and with the X-axis pointed forward.  So the Y axis reads the horizontal
  // acceleration and the inverse of the Z axis is gravity.
  // For other orientations of the sensor, just change the axis to match.
  MomentumH += TorqueH * accel.acceleration.z / swing;
  if (antiGravity)
  {
    MomentumV += TorqueV * -accel.acceleration.y / gravity;
  }
  else
  {
    MomentumV -= TorqueV * -accel.acceleration.y / gravity;
  }

  // Calculate the new position
  pos += MomentumH + MomentumV;

  // handle the wrap-arounds at the top
  while (round(pos) < 0) pos += NUM_LEDS;
  while (round(pos) > NUM_LEDS - 1) pos -= NUM_LEDS;

  int lightOn[round(pupilRadius * 2 + 1)];
  int lightIndex = 0;
  for(int i = pos; i > pos - pupilRadius; i--)
  {
    lightOn[lightIndex] = wrapAround(i);
    lightIndex++;
  }
  for(int i = pos; i < pos + pupilRadius; i++)
  {
    lightOn[lightIndex] = wrapAround(i);
    lightIndex++;
  }
  for (int i = 0; i < NUM_LEDS; i++)
  {
    int rightIndex = i;
    int leftIndex = wrapAround(i - 10); // This is because the strips aren't installed in the same place
    boolean turnedOn = false;
    for(int j = 0; j < (pupilRadius * 2 + 1); j++)
    {
      if(lightOn[j] == i)
      {
        leftLense[leftIndex] = color;
        rightLense[rightIndex] = color;
        turnedOn = true;
      }
    }
    if(turnedOn == false)
    {
      leftLense[leftIndex] = CRGB(0, 0, 0);
      rightLense[rightIndex] = CRGB(0, 0, 0);
    }
  }
  FastLED.show();

}

int wrapAround(int value)
{
  if(value < 0)
  {
    return value + NUM_LEDS;
  }
  if(value > NUM_LEDS - 1)
  {
    return value - NUM_LEDS;
  }
  return value;
}

CRGB convertHeadingToColor(float heading)
{
  int convertedHeading = map(heading, 0, 51, 0, 255);
  return CHSV(convertedHeading, 255, 180);
}

// monitor orientation for mode-change 'gestures'
void checkForGestures(sensors_event_t accel)
{
  if (accel.acceleration.x < gestureThreshold)
  {
    if (millis() - gestureStart > gestureHoldTime)
    {
      gestureStart = millis(); // reset timer
      spinDown();
      activateGestureModeSelect();
    }
  }
  else     // no nods in progress
  {
    gestureStart = millis(); // reset timer
  }
}

// Reset to default
void resetModes()
{
  antiGravity = true;
  mirroredEyes = false;

  /// spin-up
  spin(CRGB(255, 0, 0), 1, 250);
  spin(CRGB(0, 255, 0), 1, 250);
  spin(CRGB(0, 0, 255), 1, 250);
  spinUp();
}

void cycleModeForward()
{
  modeNumber++;
  Serial.print("Forward to mode : ");
  Serial.println(modeNumber);
  setDisplayMode();
}

void cycleModeBack()
{
  modeNumber--;
  Serial.print("Back to mode : ");
  Serial.println(modeNumber);
  setDisplayMode();
}

void setDisplayMode()
{
  int numberOfModes = 4;
  if(modeNumber > numberOfModes)
  {
    modeNumber = 1;
  }
  if(modeNumber < 1)
  {
    modeNumber = numberOfModes;
  }

  Serial.print("Actually switching to mode : ");
  Serial.println(modeNumber);

  switch (modeNumber)
  {
    case 1:
      pendulum = true;
      antiGravity = true;
      mirroredEyes = false;
      break;
    case 2:
      pendulum = true;
      antiGravity = false;
      mirroredEyes = true;
      break;
    case 3:
      pendulum = true;
      antiGravity = true;
      mirroredEyes = true;
      break;
    case 4: //Some preset pattern? How do?
      Serial.println("Setting pendulum to false right fucking now!");
      pendulum = false;
      break;
    default:
      Serial.println("Oops, got into the default :()");
      pendulum = true;
      antiGravity = false;
      mirroredEyes = false;
      break;
  }
}

void activateGestureModeSelect()
{
  for(int i = 0; i < NUM_LEDS; i++)
  {
    if(i % 3 == 0)
    {
      rightLense[i] = CRGB(0, 0, 255);
      leftLense[i] = CRGB(0, 0, 255);
    }
    else
    {
      rightLense[i] = CRGB(0, 0, 0);
      leftLense[i] = CRGB(0, 0, 0);
    }
  }
  FastLED.show();
  for(int i = 0; i < 100; i++)
  {
    delay(20);
    lsm.getEvent(&accel, &mag, &gyro, &temp);
    if(accel.acceleration.z > 0.70)
    {
      cycleModeForward();
      break;
    }
    if(accel.acceleration.z < -0.70)
    {
      cycleModeBack();
      break;
    }
  }

}

// gradual spin up
void spinUp()
{
  for (int i = 300; i > 0;  i -= 20)
  {
    spin(CRGB::White, 1, i);
  }
  pos = 0;
  // leave it with some momentum and let it 'coast' to a stop
  MomentumH = 3;
}

// Gradual spin down
void spinDown()
{
  for (int i = 1; i < 300; i++)
  {
    spin(CRGB::White, 1, i += 20);
  }
  // Stop it dead at the top and let it swing to the bottom on its own
  pos = 0;
  MomentumH = MomentumV = 0;
}

// utility function for feedback on mode changes.
void spin(CRGB color, int count, int time)
{
  for (int j = 0; j < count; j++)
  {
    for (int i = 0; i < NUM_LEDS; i++)
    {
      rightLense[i] = color;
      leftLense[i] = color;
      FastLED.show();
      delay(max(time / NUM_LEDS, 1));
      leftLense[i] = CRGB::Black;
      rightLense[i] = CRGB::Black;
      FastLED.show();
    }
  }
}




