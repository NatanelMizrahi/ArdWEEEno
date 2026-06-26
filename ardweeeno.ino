/*
    A swing motion tracker that keeps a score and plays sounds interactively.

    NOTES:
    * used this video/repo as a reference https://youtu.be/7VW_XVbtu9k 
    * using Kalman filter to improve angle calculation accuracy due to gyro drift 
    * periodically re-calibrates on idle state
*/

#include <Wire.h>
#include <SoftwareSerial.h>
#include <DYPlayerArduino.h>

void doReset(bool changeVoice = true);
void displayFloat(char* label, float value, bool last = false);
void shuffle(int arr[], int length);
void shuffle(uint8_t arr[], int length);

const int AUDIO_RX_PIN = 2;
const int AUDIO_TX_PIN = 3;

SoftwareSerial AudioSerial(AUDIO_RX_PIN, AUDIO_TX_PIN);
DY::Player player(&AudioSerial);

typedef struct {
  float X, Y, Z;
} coords_t;

typedef struct {
  coords_t Acc;
  coords_t Gyro;
} state_t;

typedef struct {
  float State;
  float Uncertainty;
} kalman_t;

// *****************************
// ** CONFIGURABLE PARAMETERS **
// *****************************

// Progress gain parameters
float ANGLE_PROGRESS_GAIN = 0.55;
float PROGRESS_DECAY_RATE = 0.53;
float PROGRESS_GAIN_FACTOR = 2.8;
float PROGRESS_DECAY_LEVEL_EXP = 0.12;
float D_PITCH_UPDATE_NOISE_THRESHOLD = 0.015;
float VOLUME;
float POST_IDLE_PROGRESS_DELAY_MS = 5000.0;
float POST_RESET_PROGRESS_DELAY_MS = 12000.0;
float MIN_ANGLE_PROGRESS = 7.0;
float MIN_ANGLE_IDLE_CHECK = 10.0;
float shouldPlayOnForwardOnly = 1.0;
float ON_IDLE_CALIBRATION_CHECK_INTERVAL_MS = 30000.0;
float IS_IDLE_TEST_WINDOW_MS = 10000.0;


// Each physical unit is flashed with one active voice set, selected by SWING_INDEX:
//   0 = test unit (voice 0 only)
//   1, 2 = backup units (50/50 split of all non-test voices)
//   3 = main unit (all voices)
#define SWING_INDEX 0

#if SWING_INDEX == 0
int AVAILABLE_VOICES_IN_DEVICE[] = { 0 }; // Test params
#elif SWING_INDEX == 1
int AVAILABLE_VOICES_IN_DEVICE[] = { 1, 2, 3, 4, 5, 6, 7, 8, 9 }; // Swing 1 (backup) params
#elif SWING_INDEX == 2
int AVAILABLE_VOICES_IN_DEVICE[] = { 10, 11, 12, 13, 14, 15, 16, 17, 18 }; // Swing 2 (backup) params
#elif SWING_INDEX == 3
int AVAILABLE_VOICES_IN_DEVICE[] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18 }; // Swing 3 (main) params
#endif

// Full configuration via Serial monitor: 
// 0.55,0.53,2.8,0.12,4000.0,7.0,1.0,0.015,10000
// 0.58,0.53,2.8,0.12,4000.0,7.0,1.0,0.015,10000

// *****************************

// audio constants
const int MAX_VOLUME = 30;
const int NUM_PLAYBACK_SPEEDS = 3;
const int NUM_VOICES = 19;
const int MAX_LEVELS = 11;
const int MAX_PLAYBACKS_PER_LEVEL = 24;

const int NUM_LEVELS_PER_VOICE[NUM_VOICES] PROGMEM = { 9, 10, 10, 9, 11, 10, 10, 11, 9, 9, 9, 10, 8, 9, 8, 8, 11, 10, 10 };
const int PLAYBACKS_PER_LEVEL[NUM_VOICES][MAX_LEVELS] PROGMEM = {
  { 5,  5,  5,  5,  5,  5,  5,  5,  5},         // [0] Tst (9)
  {16, 13, 14, 13, 14, 19, 13, 21,  5,  1},     // [1] YB (10)
  { 8,  9,  6,  4,  6,  8,  8,  8, 10,  2},     // [2] AP (10)
  {13, 14, 11,  8, 10,  9,  6,  4,  1},         // [3] MT (9)
  {11, 15, 14, 11,  7,  8, 11,  5, 10,  4,  1}, // [4] BA (11)
  {13, 12,  9,  7,  7,  7,  6,  7,  5,  1},     // [5] NO (10)
  { 8,  7,  9,  9,  8, 10,  9,  9,  5,  3},     // [6] MM (10)
  {12, 11, 13, 18, 17, 14, 12, 10,  8, 12,  5}, // [7] AN (11)
  {20, 15, 16, 15, 12,  7,  3,  4,  1},         // [8] JH (9)
  {11, 15, 14, 14, 12, 11,  8,  4,  1},         // [9] EH (9)
  { 7,  8,  9,  8,  7,  7,  6,  3,  3},         // [10] NA (9)
  {17, 18, 24, 23, 20, 18, 19, 18, 21, 10},     // [11] NN (10)
  {10, 12, 13, 15, 13, 14, 12,  1},             // [12] FF (8)
  { 5, 10, 12, 10, 11, 12, 13,  9,  3},         // [13] KW (9)
  { 6,  9,  9,  8,  6,  5,  4,  1},             // [14] PL (8)
  { 4,  5,  4,  7,  7,  6,  4,  3},             // [15] SS (8)
  {12, 13,  8, 10, 12,  9, 14, 12,  9, 11,  4}, // [16] YR (11)
  {10, 12, 12, 15, 16, 12, 11,  6,  7,  2},     // [17] MP (10)
  { 6,  6,  5,  7,  7,  7,  8,  6,  5,  2}      // [18] SA (10)
};

int selectedVoice, numLevelsForVoice, selectedPlayback;
float currentVolume;
int level;
bool playedMaxLevel;
bool playedAnySoundInCurrentVoice;

state_t state, offset, angle;
float progressBarValue;
float roll = 0, pitch = 0, yaw = 0;
float elapsedTimeSeconds;
// NOTE: store raw millis() snapshots as unsigned long, not float - float's 24-bit
// mantissa loses integer precision past ~4.66h of uptime, which would silently
// degrade dt/idle-timing calculations during long-running sessions.
unsigned long currentTime, previousTime;
unsigned long lastPrintPreviousTime;
float temperature;

// Easter egg: hold ~90° for 10s to lock in a specific voice
const float EASTER_EGG_ANGLE_DEG = 85.0;
const unsigned long EASTER_EGG_HOLD_MS = 10000;
const int EASTER_EGG_VOICES[] = { 0, 12, 16, 12 }; // indexed by SWING_INDEX

// UX configuration
const bool SHOULD_INCREASE_ACCEL_FULL_RANGE = true;
const bool SHOULD_INCREASE_GYRO_FULL_RANGE = false;
const bool SERIAL_PLOTTER_ENABLED = true;
const bool VERBOSE = false;
const bool SPEED_VARIATIONS_ENABLED = false;
const int CALIBRATION_SAMPLE_SIZE = 600;
const int DISPLAY_INTERVAL_MS = 500;

// IMU configuration registers and values
const int MPU = 0x68;                                                                     // MPU6050 I2C address
const float ACCEL_DIVISION_FACTOR = SHOULD_INCREASE_ACCEL_FULL_RANGE ? 8192.0 : 16384.0;  // For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
const float GYRO_DIVISION_FACTOR = 131.0;                                                 // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
const int ACCEL_CONFIG_REGISTER = 0x1C;
const int ACCEL_CONFIG_FULL_RANGE_4G = 0x08;
const int GYRO_CONFIG_REGISTER = 0x1B;
const int GYRO_CONFIG_FULL_RANGE_VAL = 0x10;  // 1000deg/s full scale (default +/- 250deg/s)


// pitch/roll angle kalman filter constants
const int ACCELEROMETER_UNCERTAINTY_STD_DEV_DEGREES = 3;
const int GYRO_ERROR_STD_DEV_DEG_PER_SEC = 4;
const int ANGLE_UNCERTAINTY_STD_DEV_DEGREES = 2;
const float KALMAN_INITIAL_ANGLE_STATE = 0;                                                                            // swing starts in a mostly leveled surface
const float KALMAN_INITIAL_ANGLE_UNCERTAINTY = ANGLE_UNCERTAINTY_STD_DEV_DEGREES * ANGLE_UNCERTAINTY_STD_DEV_DEGREES;  //

kalman_t kalmanRoll = {
  KALMAN_INITIAL_ANGLE_STATE,
  KALMAN_INITIAL_ANGLE_UNCERTAINTY
};
kalman_t kalmanPitch = {
  KALMAN_INITIAL_ANGLE_STATE,
  KALMAN_INITIAL_ANGLE_UNCERTAINTY
};

// pitch angle derivative state variables
const float DERIVATIVE_CALC_TIME_WINDOW_MS = 100.0;

unsigned long previousDerivativeUpdateTimeMs;
float previousWindowPitchAvg;
float pitchSampleSum, pitchSampleCount;
float dPitch, previousDPitch, previousPitch;

bool playedInLevel;
unsigned long resetTimeMs;
unsigned long easterEggHoldStartMs = 0;

float maxPitchAngle;
unsigned long winTimeMs;
unsigned long lastSwingTimeMs;

enum direction_change_e {
  BACKWARDS = -1,
  NO_DIRECTION_CHANGE = 0,
  FORWARD = 1
} swingDirectionChange = NO_DIRECTION_CHANGE;

bool passedMinPoint;

float* configurableParameters[] = {
  &ANGLE_PROGRESS_GAIN,
  &PROGRESS_DECAY_RATE,
  &PROGRESS_GAIN_FACTOR,
  &PROGRESS_DECAY_LEVEL_EXP,
  &VOLUME,
  &POST_RESET_PROGRESS_DELAY_MS,
  &MIN_ANGLE_PROGRESS,
  &shouldPlayOnForwardOnly,
  &D_PITCH_UPDATE_NOISE_THRESHOLD,
  &ON_IDLE_CALIBRATION_CHECK_INTERVAL_MS
};

void updateSwingDirection() {
  unsigned long currentTimeMs = millis();
  swingDirectionChange = NO_DIRECTION_CHANGE;

  if (abs(pitch) >= MIN_ANGLE_IDLE_CHECK)
    lastSwingTimeMs = currentTimeMs;

  // calculate the pitch angle derivative using the time average value in the current time window vs the previous time window and the time delta between the windows.
  if (currentTimeMs - previousDerivativeUpdateTimeMs > DERIVATIVE_CALC_TIME_WINDOW_MS) {
    float dt = currentTimeMs - previousDerivativeUpdateTimeMs;  // small delta - safe to widen to float
    previousDerivativeUpdateTimeMs = currentTimeMs;
    pitchSampleCount = (pitchSampleCount == 0) ? 1 : pitchSampleCount; // 0 div protection
    float currentWindowPitchAvg = pitchSampleSum / pitchSampleCount;
    float newDPitch = (currentWindowPitchAvg - previousWindowPitchAvg) / dt;

    // only update angle derivative above a defined threshold to reduce noise in change direction calculations
    bool shouldUpdateDPitch = abs(newDPitch) > D_PITCH_UPDATE_NOISE_THRESHOLD;
    if (shouldUpdateDPitch) {
      dPitch = newDPitch;
    }
    previousWindowPitchAvg = currentWindowPitchAvg;
    if (previousDPitch < 0 && dPitch >= 0) {
      swingDirectionChange = FORWARD;
    }
    if (previousDPitch > 0 && dPitch <= 0) {
      swingDirectionChange = BACKWARDS;
    }
    passedMinPoint = (previousPitch <= 0 && pitch >= 0) || (previousPitch >= 0 && pitch <= 0);

    previousPitch = pitch;
    pitchSampleSum = 0;
    pitchSampleCount = 0;

    if (shouldUpdateDPitch) {
      previousDPitch = dPitch;
    }
  }
  pitchSampleSum += pitch;
  pitchSampleCount++;
}

void configureSensitivity() {
  if (SHOULD_INCREASE_ACCEL_FULL_RANGE) {
    Wire.beginTransmission(MPU);
    Wire.write(ACCEL_CONFIG_REGISTER);
    Wire.write(ACCEL_CONFIG_FULL_RANGE_4G);
    Wire.endTransmission(true);
  }

  if (SHOULD_INCREASE_GYRO_FULL_RANGE) {
    Wire.beginTransmission(MPU);
    Wire.write(GYRO_CONFIG_REGISTER);
    Wire.write(GYRO_CONFIG_FULL_RANGE_VAL);
    Wire.endTransmission(true);
  }

  delay(20);
}

// This function ensures all available voices on the device are sampled before repeating for maximal variability
// Alternative simpler implementation: return random(0, numVoices);
int getRandomVoiceIndex() {
  static int numVoices = sizeof(AVAILABLE_VOICES_IN_DEVICE) / sizeof(AVAILABLE_VOICES_IN_DEVICE[0]);
  static uint8_t permutation[NUM_VOICES];
  static int currentIndex = numVoices;
  
  currentIndex++;
  if (currentIndex >= numVoices) {
    for (int i = 0; i < numVoices; i++) {
      permutation[i] = i;
    }
    shuffle(permutation, numVoices);
    currentIndex = 0;
  }
  return permutation[currentIndex];
}

void selectRandomVoice() {
  int randomIndex = getRandomVoiceIndex();
  int randomVoice = AVAILABLE_VOICES_IN_DEVICE[randomIndex];
  selectedVoice = randomVoice;
  numLevelsForVoice = pgm_read_word(&NUM_LEVELS_PER_VOICE[selectedVoice]);
}

void displayMeasurements(state_t s) {
  displayFloat("aX", s.Acc.X);
  displayFloat("aY", s.Acc.Y);
  displayFloat("aZ", s.Acc.Z);
  displayFloat("gX", s.Gyro.X);
  displayFloat("gY", s.Gyro.Y);
  displayFloat("gZ", s.Gyro.Z);
  displayFloat("aθ", angle.Acc.Y);
}

void checkShouldRecalibrateIMU() {
  // if idle (barely moved) for 30 seconds, recailbrate to remove angle drift errors
  const int IS_IDLE_CHECK_MAX_ANGLE_DELTA = 1.5;
  static float maxPitchInInterval, minPitchInInterval;
  static unsigned long nextSampleTimeMs;

  maxPitchInInterval = max(maxPitchInInterval, pitch);
  minPitchInInterval = min(minPitchInInterval, pitch);

  if (millis() >= nextSampleTimeMs) {
    if ((maxPitchInInterval - minPitchInInterval) < IS_IDLE_CHECK_MAX_ANGLE_DELTA) {
      calibrateImu();
      doReset(false);
    }
    nextSampleTimeMs = millis() + (unsigned long)ON_IDLE_CALIBRATION_CHECK_INTERVAL_MS;  // explicit cast avoids promoting millis() to float
    maxPitchInInterval = pitch;
    minPitchInInterval = pitch;
  }
}

void calibrateImu() {
  // Note that we should place the IMU flat in order to get the correct values.

  for (int i = 0; i < CALIBRATION_SAMPLE_SIZE; i++) {
    readImuData();

    // Sum all readings
    offset.Acc.X = offset.Acc.X + ((atan((state.Acc.Y) / sqrt(pow((state.Acc.X), 2) + pow((state.Acc.Z), 2))) * 180 / PI));
    offset.Acc.Y = offset.Acc.Y + ((atan(-1 * (state.Acc.X) / sqrt(pow((state.Acc.Y), 2) + pow((state.Acc.Z), 2))) * 180 / PI));

    offset.Gyro.X = offset.Gyro.X + state.Gyro.X;
    offset.Gyro.Y = offset.Gyro.Y + state.Gyro.Y;
    offset.Gyro.Z = offset.Gyro.Z + state.Gyro.Z;
  }

  // Calculate the average to get the offset value and calibrate
  offset.Acc.X = offset.Acc.X / CALIBRATION_SAMPLE_SIZE;
  offset.Acc.Y = offset.Acc.Y / CALIBRATION_SAMPLE_SIZE;

  offset.Gyro.X = offset.Gyro.X / CALIBRATION_SAMPLE_SIZE;
  offset.Gyro.Y = offset.Gyro.Y / CALIBRATION_SAMPLE_SIZE;
  offset.Gyro.Z = offset.Gyro.Z / CALIBRATION_SAMPLE_SIZE;

  if (!SERIAL_PLOTTER_ENABLED) {
    Serial.println("******* CALIBRATION OFFSET VALUES *******");
    displayMeasurements(offset);
    Serial.println("");
    Serial.println("****************************");
  }
}

void setVolume(float newVolume) {
  if (newVolume != currentVolume) {
    int normalizedVolume = floor(map((int)newVolume, 0, 100, 0, MAX_VOLUME));
    player.setVolume(normalizedVolume);
  }
}

void configurePlayer() {
  player.begin();
  setVolume(100);
  selectRandomVoice();
}

void configureIMU() {
  Wire.setClock(400000);
  Wire.begin();                 // Initialize comunication
  Wire.beginTransmission(MPU);  // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);             // Talk to the register 6B
  Wire.write(0x00);             // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);   //end the transmission

  configureSensitivity();
  calibrateImu();
}

void displayFloat(char* label, float value, bool isLast) {
  Serial.print(label);
  Serial.print(":");
  Serial.print(value);
  if (isLast)
    Serial.println();
  else
    Serial.print(",");
}

void displayState() {
  if (SERIAL_PLOTTER_ENABLED || millis() - lastPrintPreviousTime > DISPLAY_INTERVAL_MS) {
    lastPrintPreviousTime = millis();
    displayFloat("θ", pitch);
    displayFloat("LV", level * 10);
    displayFloat("💲", progressBarValue);
    displayFloat("🔁", ((int)swingDirectionChange) * 30);
    displayFloat("🎵", selectedVoice * 10);
    displayFloat("#", selectedPlayback * 10 + 5);
    
    if (VERBOSE){
      displayFloat("r", roll);
      displayFloat("y", yaw);
      displayMeasurements(state);
    } 
    
    Serial.println();
  }
}

void setup() {
  Serial.begin(9600);
  randomSeed(analogRead(A0));  // seed once from an UNCONNECTED analog pin (A4/A5 are I2C SDA/SCL - do not use)
  configureIMU();
  configurePlayer();
  delay(20);
}

float readFloat() {
  return Wire.read() << 8 | Wire.read();
}

void readImuData() {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 7 * 2, true);  // Read 14 registers total, each value is stored in 2 registers

  // For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  state.Acc.X = readFloat() / ACCEL_DIVISION_FACTOR;
  state.Acc.Y = readFloat() / ACCEL_DIVISION_FACTOR;
  state.Acc.Z = readFloat() / ACCEL_DIVISION_FACTOR;

  temperature = readFloat();

  // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  state.Gyro.X = readFloat() / GYRO_DIVISION_FACTOR;
  state.Gyro.Y = readFloat() / GYRO_DIVISION_FACTOR;
  state.Gyro.Z = readFloat() / GYRO_DIVISION_FACTOR;
}

void readParameters() {
  if (Serial.available() == 0)
    return;

  int numParams = sizeof(configurableParameters) / sizeof(configurableParameters[0]);
  float newValue;
  char delimiter;

  for (int i = 0; i < numParams; i++) {
    newValue = Serial.parseFloat();
    *configurableParameters[i] = newValue;
    delimiter = Serial.read();
    if (delimiter == ';')
      break;
  }

  while (Serial.available() > 0)
    Serial.read();

  setVolume(VOLUME);
}

kalman_t kalmanFilter(
  kalman_t kalman,
  float kalmanInputGyroRate,
  float kalmanMeasurementAccelAngle,
  float elapsedTimeSeconds) {

  float dt = elapsedTimeSeconds;
  kalman.State = kalman.State + dt * kalmanInputGyroRate;
  kalman.Uncertainty = kalman.Uncertainty + pow(dt, 2) * pow(GYRO_ERROR_STD_DEV_DEG_PER_SEC, 2);

  float kalmanGain = kalman.Uncertainty * 1 / (1 * kalman.Uncertainty + pow(ACCELEROMETER_UNCERTAINTY_STD_DEV_DEGREES, 2));
  kalman.State = kalman.State + kalmanGain * (kalmanMeasurementAccelAngle - kalman.State);
  kalman.Uncertainty = (1 - kalmanGain) * kalman.Uncertainty;
  return {
    kalman.State,
    kalman.Uncertainty
  };
}

void measureAngles() {
  previousTime = currentTime;
  currentTime = millis();
  elapsedTimeSeconds = (currentTime - previousTime) / 1000.0;  // force float division - both operands are now unsigned long
  readImuData();

  // Correct the accelerometer outputs with the calculated offset values
  // NOTE: deducting the offset directly from the angle acceleration yields better results (see below)
  // state.Acc.X = state.Acc.X - offset.Acc.X;
  // state.Acc.Y = state.Acc.Y - offset.Acc.Y;
  // state.Acc.Z = state.Acc.Z - offset.Acc.Z;

  // Calculating Roll and Pitch from the accelerometer data with the calculated offset values
  angle.Acc.X = (atan(state.Acc.Y / sqrt(pow(state.Acc.X, 2) + pow(state.Acc.Z, 2))) * 180 / PI) - offset.Acc.X;
  angle.Acc.Y = (atan(-1 * state.Acc.X / sqrt(pow(state.Acc.Y, 2) + pow(state.Acc.Z, 2))) * 180 / PI) - offset.Acc.Y;

  // Correct the gyro outputs with the calculated offset values
  state.Gyro.X = state.Gyro.X - offset.Gyro.X;
  state.Gyro.Y = state.Gyro.Y - offset.Gyro.Y;
  state.Gyro.Z = state.Gyro.Z - offset.Gyro.Z;

  // The raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds to get the angle in degrees
  angle.Gyro.X = angle.Gyro.X + state.Gyro.X * elapsedTimeSeconds;
  angle.Gyro.Y = angle.Gyro.Y + state.Gyro.Y * elapsedTimeSeconds;

  // Complementary filter - combine acceleromter and gyro angle values
  // yaw = yaw + state.Gyro.Z * elapsedTimeSeconds;
  // roll = 0.96 * angle.Gyro.X + 0.04 * angle.Acc.X;
  // pitch = 0.96 * angle.Gyro.Y + 0.04 * angle.Acc.Y;

  // using a kalman filter to reduce inaccuracy caused by gyro drift
  kalmanRoll = kalmanFilter(kalmanRoll, state.Gyro.X, angle.Acc.X, elapsedTimeSeconds);
  kalmanPitch = kalmanFilter(kalmanPitch, state.Gyro.Y, angle.Acc.Y, elapsedTimeSeconds);

  roll = kalmanRoll.State;
  pitch = kalmanPitch.State;
  yaw = yaw + state.Gyro.Z * elapsedTimeSeconds;  // can't use kalman filter for yaw

  maxPitchAngle = max(maxPitchAngle, abs(pitch));
}

bool shouldUpdateProgress() {
  if (abs(pitch) < MIN_ANGLE_PROGRESS)
    return false;
  if (millis() < resetTimeMs)
    return false;
  return true;
}

void runProgressBarErrorCorrection() {
  // if stuck due to serial error, reset progress
  if (progressBarValue == 100 && (millis() - winTimeMs) > 5000) {
    doReset();
  }
}

// the progress rate is determined by 4 configurable parameters: ANGLE_PROGRESS_GAIN, PROGRESS_GAIN_FACTOR, PROGRESS_DECAY_RATE, PROGRESS_DECAY_LEVEL_EXP
// the progress bar score $ formula is: new$ = $ + (|θ| ^ Gain) / GainFactor - (DecayRate + (1 + $) ^ DecayLevelExponent)
void updateProgressBar() {
  if (!shouldUpdateProgress())
    return;
  float gain = pow(abs(pitch), ANGLE_PROGRESS_GAIN) / PROGRESS_GAIN_FACTOR;
  float decay = PROGRESS_DECAY_RATE + pow(1 + progressBarValue, PROGRESS_DECAY_LEVEL_EXP);
  float newValue = progressBarValue + gain - decay;
  newValue = constrain(newValue, 0.0, 100.0);

  if (progressBarValue < 100 && newValue == 100) {
    winTimeMs = millis();
  }

  progressBarValue = newValue;
  runProgressBarErrorCorrection();
}

void doReset(bool changeVoice){
  progressBarValue = 0;
  maxPitchAngle = 0;
  playedMaxLevel = false;
  playedAnySoundInCurrentVoice = false;
  resetTimeMs = millis() + (unsigned long)POST_RESET_PROGRESS_DELAY_MS;  // explicit cast avoids promoting millis() to float
  if (changeVoice)
    selectRandomVoice();
}

void checkAndProcessReset() {
  if (playedMaxLevel) {
    // wait for last sound to finish playing and reset, with timout to prevent infinite loop
    unsigned long timeout = millis() + 30000UL;
    while (player.checkPlayState() == DY::PlayState::Playing && millis() < timeout) {
      delay(100);
    }
    doReset();
    return;
  }

  // switch voice if no one is swinging for a while and current voice was played at least once
  unsigned long now = millis();
  bool shouldResetOnIdle = playedAnySoundInCurrentVoice && (now - lastSwingTimeMs > IS_IDLE_TEST_WINDOW_MS);
  if (shouldResetOnIdle) {
    displayFloat("RESET", 20);
    doReset();
    resetTimeMs = now + (unsigned long)POST_IDLE_PROGRESS_DELAY_MS;  // explicit cast avoids promoting `now` to float
  }
}

bool shouldPlaySound() {
  if (millis() < resetTimeMs)
    return false;
  if ((level == 0) && (abs(pitch) < MIN_ANGLE_PROGRESS))
    return false;
  if (!(swingDirectionChange == FORWARD || !shouldPlayOnForwardOnly && swingDirectionChange == BACKWARDS))
    return false;
  if (player.checkPlayState() == DY::PlayState::Playing)
    return false;
  return true;
}

void shuffle(int arr[], int length) {
  // NOTE: the RNG is seeded once in setup(); do NOT reseed here (it would repeat permutations)
  for (int i = length - 1; i > 0; i--) {
    int j = random(0, i + 1);
    int temp = arr[i];
    arr[i] = arr[j];
    arr[j] = temp;
  }
}

void shuffle(uint8_t arr[], int length) {
  for (int i = length - 1; i > 0; i--) {
    int j = random(0, i + 1);
    uint8_t temp = arr[i];
    arr[i] = arr[j];
    arr[j] = temp;
  }
}

// this function ensures all voices in each level are sampled before repeating for maximal variability
// it randomizes the sound order and keeps a bookmark of each level's played indexes
int getLevelRandomPermutationIndex(int level) {
  static uint8_t soundPermutationByLevel[MAX_LEVELS][MAX_PLAYBACKS_PER_LEVEL];
  static uint8_t indexByLevel[MAX_LEVELS];
  static int8_t randomizedForVoice[MAX_LEVELS];  // which voice each cached permutation was built for (-1 = none)

  static bool initialized = false;
  if (!initialized) {
    for (int i = 0; i < MAX_LEVELS; i++)
      randomizedForVoice[i] = -1;  // can't default-init to 0, since 0 is a valid voice index
    initialized = true;
  }

  int numLevelOptions = pgm_read_word(&PLAYBACKS_PER_LEVEL[selectedVoice][level]);
  if (numLevelOptions <= 0)
    return 0;  // no playbacks defined for this voice/level

  int* permutation = soundPermutationByLevel[level];

  // reshuffle when the voice changed (cached permutation is stale) or the cycle is exhausted.
  // use >= (not ==) so we still recover if indexByLevel ever overshoots numLevelOptions.
  if (randomizedForVoice[level] != selectedVoice || indexByLevel[level] >= numLevelOptions) {
    for (int i = 0; i < numLevelOptions; i++) {
      permutation[i] = i;
    }
    shuffle(permutation, numLevelOptions);
    indexByLevel[level] = 0;
    randomizedForVoice[level] = selectedVoice;
  }

  return permutation[indexByLevel[level]++];
}

void play(int voice, int level, int index, int speed) {
  static char path[30];

  if (SPEED_VARIATIONS_ENABLED)
    sprintf(path, "/%d/%d/%d/%d.wav", voice, level, index, speed);
  else
    sprintf(path, "/%d/%d_%d.WAV", voice, level, index);

  player.playSpecifiedDevicePath(DY::Device::Sd, path);
}

void playSound() {
  level = floor(map(progressBarValue, 0, 100, 0, numLevelsForVoice - 1));
  if (!shouldPlaySound())
    return;

  int randomPlaybackIndex = getLevelRandomPermutationIndex(level);
  int randomSpeed = random(0, NUM_PLAYBACK_SPEEDS);
  selectedPlayback = randomPlaybackIndex;
  
  play(selectedVoice, level, randomPlaybackIndex, randomSpeed);
  playedAnySoundInCurrentVoice = true;
  playedMaxLevel = (level == (numLevelsForVoice - 1));
}

void checkEasterEgg() {
  if (abs(pitch) >= EASTER_EGG_ANGLE_DEG) {
    if (easterEggHoldStartMs == 0)
      easterEggHoldStartMs = millis();
    else if (millis() - easterEggHoldStartMs >= EASTER_EGG_HOLD_MS) {
      selectedVoice = EASTER_EGG_VOICES[SWING_INDEX];
      numLevelsForVoice = pgm_read_word(&NUM_LEVELS_PER_VOICE[selectedVoice]);
      easterEggHoldStartMs = 0;
      play(0, 0, 1, 0); // confirmation sound
      doReset(false);
    }
  } else {
    easterEggHoldStartMs = 0;
  }
}

void loop() {
  readParameters();
  measureAngles();
  checkEasterEgg();
  updateSwingDirection();
  updateProgressBar();
  displayState();
  playSound();
  checkAndProcessReset();
  checkShouldRecalibrateIMU();
}
