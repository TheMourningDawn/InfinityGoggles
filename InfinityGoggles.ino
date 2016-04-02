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

//FASTLED_USING_NAMESPACE;

#define DATAPIN_LEFT    0
#define CLOCKPIN_LEFT   1
#define DATAPIN_RIGHT    6
#define CLOCKPIN_RIGHT   9
#define NUM_LEDS 21 //Number of LEDS per lense

#define FRAMES_PER_SECOND  5
#define BRIGHTNESS         120

#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))

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
const float pupilRadius = 2; // half-width of pupil (in pixels)

long gestureStart = 0;
long gestureHoldTime = 1500;

uint8_t hue = 0;
uint8_t currentPatternNumber = 0; // Index number of which pattern is current


// List of patterns to cycle through.  Each is defined as a separate function below.
typedef void (*SimplePattern)();

typedef void (*Action)();

typedef struct {
    SimplePattern pattern;
    Action action;
} PatternDefinition;
typedef PatternDefinition PatternDefinitionList[];

const PatternDefinitionList patterns = {
        {nothing,       pendulum},
        {nothing,       pendulumAntiGravity},
        {nothing,       bpm},
        {nothing,       juggle},
        {nothing,       sinelon},
        {lowKeyRainbow, rotateClockwise},
        {meteor,        rotateClockwise}
};


void setup(void) {
    FastLED.addLeds<APA102, DATAPIN_LEFT, CLOCKPIN_LEFT>(leftLense, NUM_LEDS).setCorrection(TypicalLEDStrip);
    FastLED.addLeds<APA102, DATAPIN_RIGHT, CLOCKPIN_RIGHT>(rightLense, NUM_LEDS).setCorrection(TypicalLEDStrip);
    FastLED.setBrightness(BRIGHTNESS);

    // Try to initialise the 9DOF
    if (!lsm.begin()) {
        while (1);
    }

    // Intitialize the sensors and start with default pattern
    setupSensor();
    patterns[currentPatternNumber].pattern();
}

void setupSensor() {
    // 1.) Set the accelerometer range
    lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G); // 2G, 4G, 6G, 8G, 16G
    // 2.) Set the magnetometer sensitivity
    lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS); //2GAUSS,4GAUSS,8GAUSS,12GAUSS
    // 3.) Setup the gyroscope. Options are: 245DPS,500DPS,2000DPS
    lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
}

void loop(void) {
    // Read the magnetometer and determine the compass heading:
    lsm.getEvent(&accel, &mag, &gyro, &temp);
    // Check for mode change commands
    checkForGestures(accel);

    // Call the current pattern function once, updating the 'leds' array
    patterns[currentPatternNumber].action();
}

void nextPattern() {
    currentPatternNumber = wrapAround(currentPatternNumber + 1, ARRAY_SIZE(patterns));
    patterns[currentPatternNumber].pattern();
}

void previousPattern() {
    currentPatternNumber = wrapAround(currentPatternNumber - 1, ARRAY_SIZE(patterns));
    patterns[currentPatternNumber].pattern();
}

//Constrains a number to to range 0-maxValue exclusive
int wrapAround(int value) {
    return wrapAround(value, NUM_LEDS);
}

int wrapAround(int value, int maxValue) {
    if (value < 0) {
        return value + maxValue;
    }
    if (value > NUM_LEDS - 1) {
        return value - maxValue;
    }
    return value;
}

CRGB convertHeadingToColor(float heading, int brightness) {
    int convertedHeading = map(heading, 0, 51, 0, 255);
    return CHSV(convertedHeading, 255, brightness);
}

// monitor orientation for mode-change 'gestures'
void checkForGestures(sensors_event_t accel) {
    if (accel.acceleration.x < gestureThreshold) {
        if (millis() - gestureStart > gestureHoldTime) {
            gestureStart = millis(); // reset timer
            activateGestureModeSelect();
        }
    }
    else {
        gestureStart = millis(); // reset timer
    }
}

void activateGestureModeSelect() {
    uint8_t hueCounter = 1;
    clearStrip();
    FastLED.show();
    int ledsPerSection = round(NUM_LEDS / ARRAY_SIZE(patterns));
    for (int i = 0; i < currentPatternNumber + 1; i++) {
        int sectionStartPixel = i * ledsPerSection;
        turnSectionOn(CHSV(hueCounter, 200, 100), ledsPerSection, sectionStartPixel);
        FastLED.show();
        hueCounter += round(255 / ARRAY_SIZE(patterns));
        delay(250);
    }
    for (int i = 0; i < 100; i++) {
        delay(25);
        lsm.getEvent(&accel, &mag, &gyro, &temp);
        if (accel.acceleration.z > 0.70) {
            turnSectionOff(ledsPerSection, ledsPerSection * (currentPatternNumber));
            if (currentPatternNumber - 1 == 255) {
                clearStrip();
            }
            blinkSection(CHSV(hueCounter - 2 * round(255 / ARRAY_SIZE(patterns)), 200, 100), ledsPerSection,
                         ledsPerSection * (currentPatternNumber - 1), 5, 150);
            previousPattern();
            break;
        }
        if (accel.acceleration.z < -0.70) {
            if (currentPatternNumber + 1 == ARRAY_SIZE(patterns)) {
                clearStrip();
            }
            blinkSection(CHSV(hueCounter + round(255 / ARRAY_SIZE(patterns)), 200, 100), ledsPerSection,
                         ledsPerSection * (currentPatternNumber + 1), 5, 150);
            nextPattern();
            break;
        }
    }
}


/********************************************************************
 * ACTIONS:
 * These are movement patterns that alter the existing pattern in
 * some way. All can be used as an 'action' type
 *******************************************************************/
//TODO:Check out the memmove function to maybe do it more quickly
//Also, if we dont want to pass the size, could technically do sizeof(strip)/sizeof(CRGB)
void rotateClockwise() {
    shift(leftLense, NUM_LEDS, false);
    shift(rightLense, NUM_LEDS, false);
    delay(30);
    FastLED.show();
}

void rotateCounterClockwise() {
    shift(leftLense, NUM_LEDS, true);
    shift(rightLense, NUM_LEDS, true);
    delay(30);
    FastLED.show();
}

void shift(CRGB strip[], int stripLength, bool changeDirection) {
    CRGB wrapAroundPixel;
    if (changeDirection == true) {
        wrapAroundPixel = strip[stripLength - 1];
        for (int i = stripLength - 1; i > 0; i--) {
            strip[i] = strip[i - 1];
        }
        strip[0] = wrapAroundPixel;
    }
    else {
        wrapAroundPixel = strip[0];
        for (int i = 0; i < stripLength; i++) {
            strip[i] = strip[i + 1];
        }
        strip[stripLength - 1] = wrapAroundPixel;
    }
}

// random colored speckles that blink in and fade smoothly...but it sucks
void confetti() {
    fadeToBlackBy(leftLense, NUM_LEDS, 25);
    fadeToBlackBy(rightLense, NUM_LEDS, 25);
    uint8_t pos = random16(NUM_LEDS);

    leftLense[pos] += CHSV(hue + random8(64), 200, 200);
    rightLense[pos] += CHSV(hue + random8(64), 200, 200);

    FastLED.show();
    FastLED.delay(1000 / FRAMES_PER_SECOND);
    hue++;
    EVERY_N_MILLISECONDS(20)
    { hue++; }
}

void bpm() {
    // colored stripes pulsing at a defined Beats-Per-Minute (BPM)
    uint8_t BeatsPerMinute = 120;
    CRGBPalette16 palette = PartyColors_p;
    uint8_t beat = beatsin8(BeatsPerMinute, 64, 255);
    for (int i = 0; i < NUM_LEDS; i++) { //9948
        rightLense[i] = ColorFromPalette(palette, hue + (i * 2), beat - hue + (i * 8));
        leftLense[i] = ColorFromPalette(palette, hue + (i * 2), beat - hue + (i * 8));
    }
    FastLED.show();
    EVERY_N_MILLISECONDS( 20 ) { hue++; } // slowly cycle the "base color" through the rainbow
}

void sinelon()
{
    // a colored dot sweeping back and forth, with fading trails
    fadeToBlackBy( rightLense, NUM_LEDS, 20);
    fadeToBlackBy( leftLense, NUM_LEDS, 20);
    int pos = beatsin16(13,0,NUM_LEDS);
    rightLense[pos] += CHSV( hue, 200, 192);
    leftLense[pos] += CHSV( hue, 200, 192);

    FastLED.show();
    EVERY_N_MILLISECONDS( 20 ) { hue++; } // slowly cycle the "base color" through the rainbow
}


void juggle() {
    // eight colored dots, weaving in and out of sync with each other
    fadeToBlackBy( rightLense, NUM_LEDS, 20);
    fadeToBlackBy( leftLense, NUM_LEDS, 20);
    byte dothue = 0;
    for( int i = 0; i < 8; i++) {
        rightLense[beatsin16(i+7,0,NUM_LEDS)] |= CHSV(dothue, 200, 180);
        leftLense[beatsin16(i+7,0,NUM_LEDS)] |= CHSV(dothue, 200, 180);
        dothue += 32;
    }

    FastLED.show();
    EVERY_N_MILLISECONDS( 20 ) { hue++; } // slowly cycle the "base color" through the rainbow
}

void pendulum() {
    CRGB magHeadingBasedColor = convertHeadingToColor(fabs(round(mag.magnetic.z * 100)), 180);
    pendulumMode(magHeadingBasedColor, false, false);
}

void pendulumAntiGravity() {
    CRGB magHeadingBasedColor = convertHeadingToColor(fabs(round(mag.magnetic.z * 100)), 180);
    pendulumMode(magHeadingBasedColor, true, false);
}

void pendulumMirrored() {
    CRGB magHeadingBasedColor = convertHeadingToColor(fabs(round(mag.magnetic.z * 100)), 180);
    pendulumMode(magHeadingBasedColor, false, true);
}

void pendulumMode(CRGB color, bool antiGravity, bool mirroredEyes) {
    // apply a little frictional damping to keep things in control and prevent perpetual motion
    MomentumH *= friction;
    MomentumV *= friction;

    // Calculate the horizontal and vertical effect on the virtual pendulum
    // 'pos' is a pixel address, so we multiply by 'increment' to get radians.
    float TorqueH = cos(pos * increment);  // peaks at top and bottom of the swing
    float TorqueV = sin(pos * increment);   // peaks when the pendulum is horizontal

    // Add the incremental acceleration to the existing momentum
    // This code assumes that the accelerometer is mounted upside-down, level
    // and with the X-axis pointed forward.  So the Y axis reads the horizontal
    // acceleration and the inverse of the Z axis is gravity.
    // For other orientations of the sensor, just change the axis to match.
    MomentumH += TorqueH * accel.acceleration.z / swing;
    if (antiGravity) {
        MomentumV += TorqueV * -accel.acceleration.y / gravity;
    }
    else {
        MomentumV -= TorqueV * -accel.acceleration.y / gravity;
    }

    // Calculate the new position
    pos += MomentumH + MomentumV;

    // handle the wrap-arounds
    while (round(pos) < 0) pos += NUM_LEDS;
    while (round(pos) > NUM_LEDS - 1) pos -= NUM_LEDS;

    int lightOn[round(pupilRadius * 2 + 1)];
    int lightIndex = 0;
    for (int i = pos; i > pos - pupilRadius; i--) {
        lightOn[lightIndex] = wrapAround(i);
        lightIndex++;
    }
    for (int i = pos; i < pos + pupilRadius; i++) {
        lightOn[lightIndex] = wrapAround(i);
        lightIndex++;
    }
    for (int i = 0; i < NUM_LEDS; i++) {
        int rightIndex = i;
        int leftIndex = wrapAround(i - 10); // This is because the strips aren't installed in the same place
        boolean turnedOn = false;
        for (int j = 0; j < (pupilRadius * 2 + 1); j++) {
            if (lightOn[j] == i) {
                leftLense[leftIndex] = color;
                rightLense[rightIndex] = color;
                turnedOn = true;
            }
        }
        if (turnedOn == false) {
            leftLense[leftIndex] = CRGB(0, 0, 0);
            rightLense[rightIndex] = CRGB(0, 0, 0);
        }
    }
    FastLED.show();
}

int brightness = 180;
bool swap = true;
bool direction = false;

void pulse(CRGB strip[], int center, int radius) {
    if (swap == true) {
        swap = false;
    }
    int fadeAmount = 3;
    if (direction == false) {
        fadeAmount = -fadeAmount;
    }
    lsm.getEvent(&accel, &mag, &gyro, &temp);
    CRGB magHeadingBasedColor = convertHeadingToColor(fabs(round(mag.magnetic.z * 100)), 180);
    int tempBrightness = brightness;

    strip[center] = magHeadingBasedColor;
    strip[center].nscale8_video(brightness);
    for (int i = center + 1; i <= center + radius; i++) {
        tempBrightness -= 5;
        if (tempBrightness < 0) {
            tempBrightness = 0;
        }
        strip[wrapAround(i)] = strip[wrapAround(i - 1)];
        strip[wrapAround(i)].nscale8_video(tempBrightness);
    }
    tempBrightness = brightness;
    for (int i = center - 1; i >= center - radius; i--) {
        tempBrightness -= 5;
        if (tempBrightness < 0) {
            tempBrightness = 0;
        }
        strip[wrapAround(i)] = strip[wrapAround(i + 1)];
        strip[wrapAround(i)].nscale8_video(tempBrightness);
    }

    FastLED.show();

    brightness = brightness + fadeAmount;
    if (brightness == 0 || brightness == 255) {
        if (direction == false) { direction = true; }
        else if (direction == true) { direction = false; }
        if (brightness == 0) {
            if (swap == false) { swap = true; }
            else if (swap == true) { swap = false; }
            clearStrip();
        }
    }
}

/********************************************************************
 * SimplePatterns:
 * These are static patterns that rely on a method like shift to
 * move them around one step at a time. As to not busy the
 * arduino from checking for sensor input.
 *******************************************************************/
//TODO: Just use clearStrip instead...? or maybe this'll be useful?
void nothing() {
    //Do nothing...
}

void clearStrip() {
    for (int i = 0; i < NUM_LEDS; i++) {
        leftLense[i] = CRGB(0, 0, 0);
        rightLense[i] = CRGB(0, 0, 0);
    }
}

void lowKeyRainbow() {
    for (int i = 0; i < NUM_LEDS; i++) {
        leftLense[i] = CHSV(12.75 * i, 255, 60);
        rightLense[i] = CHSV(12.75 * i, 255, 60);
    }
    FastLED.show();
}

void meteor() {
    clearStrip();
    meteor(leftLense, 0, 9, 200);
    meteor(rightLense, 0, 9, 200);
    FastLED.show();
}

void meteor(CRGB strip[], int meteorBodyPixel, int tailLength, int fade) {
    int fadeSpectrum = fade;
    int fadeIncrement = (256 - fade) / tailLength;

    lsm.getEvent(&accel, &mag, &gyro, &temp);
    CRGB magHeadingBasedColor = convertHeadingToColor(fabs(round(mag.magnetic.z * 100)), 180);

    for (int i = 0; i < tailLength; i++) {
        strip[i] = magHeadingBasedColor;
        strip[i].fadeLightBy(fadeSpectrum);
        fadeSpectrum += fadeIncrement;
    }
}


/********************************************************************
 * These animations that hold up the loop, so you wont be able to
 * check for gestures, or anything of the such.
 *******************************************************************/

// gradual spin up
void spinUp() {
    for (int i = 300; i > 0; i -= 20) {
        spin(CRGB::White, 1, i);
    }
    pos = 0;
    // leave it with some momentum and let it 'coast' to a stop
    MomentumH = 3;
}

// Gradual spin down
void spinDown() {
    for (int i = 1; i < 300; i += 20) {
        spin(CRGB::White, 1, i);
    }
    // Stop it dead at the top and let it swing to the bottom on its own
    pos = 0;
    MomentumH = MomentumV = 0;
}

// utility function for feedback on mode changes.
void spin(CRGB color, int count, int time) {
    for (int j = 0; j < count; j++) {
        for (int i = 0; i < NUM_LEDS; i++) {
            rightLense[i] = color;
            leftLense[i] = color;
            rightLense[i].fadeLightBy(180);
            leftLense[i].fadeLightBy(180);
            FastLED.show();
            delay(max(time / NUM_LEDS, 1));
            leftLense[i] = CRGB::Black;
            rightLense[i] = CRGB::Black;
            FastLED.show();
        }
    }
}

void blinkSection(CRGB color, int count, int startPixel, int numberOfBlinks, int blinkDelay) {
    for (int i = 0; i < numberOfBlinks; i++) {
        turnSectionOn(color, count, startPixel);
        FastLED.show();
        delay(blinkDelay);
        turnSectionOff(count, startPixel);
        FastLED.show();
        delay(blinkDelay);
    }
}

void turnSectionOn(CRGB color, int count, int startPixel) {
    for (int i = startPixel; i < startPixel + count; i++) {
        int rightIndex = wrapAround(i);
        int leftIndex = wrapAround(i - 10);
        rightLense[rightIndex] = color;
        leftLense[leftIndex] = color;
        if (rightIndex + 1 == NUM_LEDS - 1) {
            rightLense[rightIndex + 1] = color;
            leftLense[wrapAround(leftIndex + 1)] = color;
        }
    }
}

void turnSectionOff(int count, int startPixel) {
    for (int i = startPixel; i < startPixel + count; i++) {
        int rightIndex = wrapAround(i);
        int leftIndex = wrapAround(i - 10);
        rightLense[rightIndex] = CRGB::Black;
        leftLense[leftIndex] = CRGB::Black;
        if (rightIndex + 1 == NUM_LEDS - 1) {
            rightLense[rightIndex + 1] = CRGB::Black;
            leftLense[wrapAround(leftIndex + 1)] = CRGB::Black;
        }
    }
    FastLED.show();
}