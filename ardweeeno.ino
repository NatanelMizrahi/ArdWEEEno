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

void displayFloat(char* label, float value, bool last = false);
void shuffle(int arr[], int length);

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
  float speed;
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
float IDLE_PROGRESS_DELAY_MS = 5000.0;
float RESET_PROGRESS_DELAY_MS = 12000.0;
float MIN_ANGLE_PROGRESS = 7.0;
float shouldPlayOnForwardOnly = 1.0;
float IS_IDLE_CHECK_INTERVAL_MS = 30000;

// int AVAILABLE_VOICES_IN_DEVICE[1] = { 10 }; // Test params
// int AVAILABLE_VOICES_IN_DEVICE[5] = { 0, 1, 2, 3, 4 }; // Swing 1 params 
int AVAILABLE_VOICES_IN_DEVICE[5] = { 5, 6, 7, 8, 9 }; // Swing 2 params 

// Full configuration via Serial monitor: 
// 0.55,0.53,2.8,0.12,4000.0,7.0,1.0,0.015,10000
// 0.58,0.53,2.8,0.12,4000.0,7.0,1.0,0.015,10000

// *****************************

// audio constants
const int MAX_VOLUME = 30;
const int NUM_PLAYBACK_SPEEDS = 3;
const int NUM_VOICES = 11;
const int MAX_LEVELS = 11;

const int NUM_LEVELS_PER_VOICE[NUM_VOICES] = { 9, 11, 10, 10, 9, 11, 9, 9, 8, 9, 9 };
const int PLAYBACKS_PER_LEVEL[NUM_VOICES][MAX_LEVELS] = {
  { 5,  5,  5,  5,  5,  5,  5,  5,  5},         // [0] Tst (9)
  {11, 15, 14, 11,  7,  8, 11,  5, 10,  4,  1}, // [1] BA (11)
  { 8,  9,  6,  4,  6,  8,  8,  8, 10,  2},     // [2] AP (10)
  { 8,  7,  9,  9,  8, 10,  9,  9,  5,  3},     // [3] MM (10)
  {13, 14, 11,  8, 10,  9,  6,  4,  1},         // [4] MT (9)
  {12, 11, 13, 18, 17, 14, 12, 10,  8, 12,  5}, // [5] AN (11)
  {13, 12,  9,  7,  7,  7,  6,  7,  5},         // [6] NO (9)
  {16, 13, 14, 13, 14, 19, 13, 21,  5},         // [7] YB (9)
  {11, 15, 14, 14, 12, 11,  8,  4},             // [8] EH (8)
  { 7,  8,  9,  8,  7,  7,  6,  3,  3},         // [9] NA (9)
  {20, 15, 16, 15, 12,  7,  3,  4,  1}          // [10] JH (9)
};

int selectedVoice, numLevelsForVoice;
float currentVolume;
int level;
bool playedMaxLevel;

state_t state, offset, angle;
float progressBarValue;
float roll = 0, pitch = 0, yaw = 0;
float elapsedTimeSeconds, currentTime, previousTime;
float lastPrintPreviousTime;
float temperature;

// UX configuration
float availableVoicesVector;
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
const float KALMAN_INITIAL_ANGLE_STATE = 0; // swing starts in a mostly leveled surface
const float KALMAN_INITIAL_ANGLE_UNCERTAINTY = ANGLE_UNCERTAINTY_STD_DEV_DEGREES * ANGLE_UNCERTAINTY_STD_DEV_DEGREES;

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

float previousDerivativeUpdateTimeMs;
float previousWindowPitchAvg;
float pitchSampleSum, pitchSampleCount;
float dPitch, previousDPitch, previousPitch;

bool playedInLevel;
float resetTimeMs;
float maxPitchAngle;
long winTimeMs;

enum direction_change_e {
  BACKWARDS = -1,
  NO_DIRECTION_CHANGE = 0,
  FORWARD = 1
} swingDirectionChange = NO_DIRECTION_CHANGE;

bool passedMinPoint;
bool isSwinging;

float* configurableParameters[] = {
  &ANGLE_PROGRESS_GAIN,
  &PROGRESS_DECAY_RATE,
  &PROGRESS_GAIN_FACTOR,
  &PROGRESS_DECAY_LEVEL_EXP,
  &VOLUME,
  &RESET_PROGRESS_DELAY_MS,
  &MIN_ANGLE_PROGRESS,
  &shouldPlayOnForwardOnly,
  &D_PITCH_UPDATE_NOISE_THRESHOLD,
  &IS_IDLE_CHECK_INTERVAL_MS
};

void updateSwingDirection() {
  float currentTimeMs = millis();
  swingDirectionChange = NO_DIRECTION_CHANGE;
  isSwinging = maxPitchAngle >= MIN_ANGLE_PROGRESS;

  // calculate the pitch angle derivative using the time average value in the current time window vs the previous time window and the time delta between the windows.
  if (currentTimeMs - previousDerivativeUpdateTimeMs > DERIVATIVE_CALC_TIME_WINDOW_MS) {
    float dt = currentTimeMs - previousDerivativeUpdateTimeMs;
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
  static int permutation[NUM_VOICES];
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
  numLevelsForVoice = NUM_LEVELS_PER_VOICE[selectedVoice];
  return randomVoice;
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
  static long nextSampleTimeMs;

  maxPitchInInterval = max(maxPitchInInterval, pitch);
  minPitchInInterval = min(minPitchInInterval, pitch);

  if (millis() >= nextSampleTimeMs) {
    if ((maxPitchInInterval - minPitchInInterval) < IS_IDLE_CHECK_MAX_ANGLE_DELTA) {
      calibrateImu();
      doReset();
    }
    nextSampleTimeMs = millis() + IS_IDLE_CHECK_INTERVAL_MS;
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
  configureIMU();
  configurePlayer();
  delay(20);
}

float readFloat() {
  return Wire.read() << 8 | Wire.read();
}

void readImuData() {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 7 * 2, true); // Read 14 registers total, each value is stored in 2 registers

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
  elapsedTimeSeconds = (currentTime - previousTime) / 1000;
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


void doReset() {
  progressBarValue = 0;
  maxPitchAngle = 0;
  playedMaxLevel = false;
  resetTimeMs = millis() + RESET_PROGRESS_DELAY_MS;
  selectRandomVoice();
}

void checkAndProcessReset() {
  if (playedMaxLevel) {
    // wait for last sound to finish playing and reset. 
    while (player.checkPlayState() == DY::PlayState::Playing);
    doReset();
  }

  if (!isSwinging) {
    resetTimeMs = millis() + IDLE_PROGRESS_DELAY_MS;
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
  randomSeed(analogRead(5));  // Seed the random number generator with noise from an unconnected analog pin
  for (int i = length - 1; i > 0; i--) {
    int j = random(0, i + 1);
    int temp = arr[i];
    arr[i] = arr[j];
    arr[j] = temp;
  }
}

// this function ensures all voices in each level are sampled before repeating for maximal variability
int getLevelRandomPermutationIndex(int level) {
  static int permutation[25];
  static int numLevelOptions;
  static int currentIndex;
  static int currentLevel = -1;

  currentIndex++;
  if (level != currentLevel || currentIndex == numLevelOptions) {
    numLevelOptions = PLAYBACKS_PER_LEVEL[selectedVoice][level];
    for (int i = 0; i < numLevelOptions; i++) {
      permutation[i] = i;
    }
    shuffle(permutation, numLevelOptions);
    currentIndex = 0;
    currentLevel = level;
  }

  return permutation[currentIndex];
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
  play(selectedVoice, level, randomPlaybackIndex, randomSpeed);
  playedMaxLevel = (level == (numLevelsForVoice - 1));
}

void loop() {
  readParameters();
  measureAngles();
  updateSwingDirection();
  updateProgressBar();
  displayState();
  playSound();
  checkAndProcessReset();
  checkShouldRecalibrateIMU();
}