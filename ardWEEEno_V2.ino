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

namespace utils
{
  void shuffle(int arr[], int length)
  {
    randomSeed(analogRead(5)); // Seed the random number generator with noise from an unconnected analog pin
    for (int i = length - 1; i > 0; i--)
    {
      int j = random(0, i + 1);
      int temp = arr[i];
      arr[i] = arr[j];
      arr[j] = temp;
    }
  }

  void displayFloat(char *label, float value, bool isLast = false)
  {
    Serial.print(label);
    Serial.print(":");
    Serial.print(value);
    if (isLast)
      Serial.println();
  }
}

class AudioController
{
private:
  static constexpr int AUDIO_RX_PIN = 2;
  static constexpr int AUDIO_TX_PIN = 3;

  // audio constants
  static constexpr int MAX_VOLUME = 30;
  static constexpr int NUM_PLAYBACK_SPEEDS = 3;
  static constexpr int NUM_VOICES = 4;
  static constexpr int MAX_LEVELS = 10;
  const int AVAILABLE_VOICES_IN_DEVICE[2] = {3, 3};
  const int NUM_LEVELS_PER_VOICE[NUM_VOICES] = {9, 9, 10, 10};
  const int PLAYBACKS_PER_LEVEL[NUM_VOICES][MAX_LEVELS] = {
      {5, 5, 5, 5, 5, 5, 5, 5, 5},     // Test
      {7, 8, 9, 8, 7, 7, 6, 3, 3},     // N
      {8, 8, 7, 4, 6, 8, 8, 8, 10, 2}, // A
      {8, 7, 9, 9, 8, 10, 9, 9, 5, 3}  // M
  };

  SoftwareSerial audioSerial;
  DY::Player audioPlayer;

  // audio status tracking variables
  int selectedVoice, numLevelsForVoice;
  float currentVolume;

  void initializePlayer()
  {
    audioPlayer.begin();
    setVolume(100);
    selectRandomVoice();
  }

  // this function ensures all voice paybacks in each level are sampled before repeating for maximal variability
  int getLevelRandomPermutationIndex(int level)
  {
    static int playbackOrderPermutation[20];
    static int numLevelOptions;
    static int currentIndex;
    static int currentLevel = -1;

    currentIndex++;
    if (level != currentLevel || currentIndex == numLevelOptions)
    {
      numLevelOptions = PLAYBACKS_PER_LEVEL[selectedVoice][level];
      for (int i = 0; i < numLevelOptions; i++)
      {
        playbackOrderPermutation[i] = i;
      }
      utils::shuffle(playbackOrderPermutation, numLevelOptions);
      currentIndex = 0;
      currentLevel = level;
    }

    return playbackOrderPermutation[currentIndex];
  }

  void play(int voice, int level, int index, int speed)
  {
    static char path[30];
    sprintf(path, "/%d/%d/%d/%d.wav", voice, level, index, speed);
    audioPlayer.playSpecifiedDevicePath(DY::Device::Sd, path);
  }

public:
  AudioController()
      : audioSerial(AUDIO_RX_PIN, AUDIO_TX_PIN),
        audioPlayer(&audioSerial)
  {
    initializePlayer();
  }

  bool tryPlaySound(int level)
  {
    if (audioPlayer.checkPlayState() == DY::PlayState::Playing)
      return false;
    int randomPlaybackIndex = getLevelRandomPermutationIndex(level);
    int randomSpeed = random(0, NUM_PLAYBACK_SPEEDS);
    play(selectedVoice, level, randomPlaybackIndex, randomSpeed);

    bool playedMaxLevel = (level == (numLevelsForVoice - 1));
    return playedMaxLevel;
  }

  void selectRandomVoice()
  {
    int numVoices = sizeof(AVAILABLE_VOICES_IN_DEVICE) / sizeof(AVAILABLE_VOICES_IN_DEVICE[0]);
    int randomIndex = random(0, numVoices);
    int randomVoice = AVAILABLE_VOICES_IN_DEVICE[randomIndex];
    selectedVoice = randomVoice;
    numLevelsForVoice = NUM_LEVELS_PER_VOICE[selectedVoice];
    return randomVoice;
  }

  void setVolume(float newVolume)
  {
    if (newVolume != currentVolume)
    {
      int normalizedVolume = floor(map((int)newVolume, 0, 100, 0, MAX_VOLUME));
      audioPlayer.setVolume(normalizedVolume);
    }
  }

  int getNumLevels() const
  {
    return numLevelsForVoice;
  }
};

class SwingMotionTracker
{
private:
  // Configuration constants
  static constexpr float DEFAULT_ANGLE_PROGRESS_GAIN = 0.55;
  static constexpr float DEFAULT_PROGRESS_DECAY_RATE = 0.53;
  static constexpr float DEFAULT_PROGRESS_GAIN_FACTOR = 2.8;
  static constexpr float DEFAULT_PROGRESS_DECAY_EXP = 0.12;
  static constexpr float DEFAULT_MIN_ANGLE_PROGRESS = 7.0;
  static constexpr float DEFAULT_RESET_PROGRESS_DELAY_MS = 20000.0;
  static constexpr float DEFAULT_PITCH_UPDATE_NOISE_THRESHOLD = 0.015;
  static constexpr float DEFAULT_IDLE_CHECK_INTERVAL_MS = 30000.0;

  // IMU and Kalman filter configuration
  static constexpr int MPU_ADDRESS = 0x68;
  static constexpr int CALIBRATION_SAMPLE_SIZE = 600;
  static constexpr float ACCEL_DIVISION_FACTOR = 8192.0; // For a range of +-4g, we need to divide the raw values by 8192, according to the datasheet
  static constexpr float GYRO_DIVISION_FACTOR = 131.0;   // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet

  // Struct definitions
  struct Coordinates
  {
    float X, Y, Z;
  };

  struct State
  {
    Coordinates Acc;
    Coordinates Gyro;
  };

  struct KalmanState
  {
    float State;
    float Uncertainty;
  };

  // Member variables
  AudioController audioController;
  State state;
  State offset;
  Coordinates accelAngle;
  KalmanState kalmanRoll;
  KalmanState kalmanPitch;

  // Swing motion tracking variables
  int level;
  float progressBarValue = 0;
  float winTimeMs = 0;
  bool playedMaxLevel = false;

  float roll, pitch, yaw;
  float maxPitchAngle = 0;
  float currentTime, previousTime, elapsedTimeSeconds;
  float resetTimeMs = 0;
  bool isSwinging;

  enum DirectionChange
  {
    BACKWARDS = -1,
    NO_DIRECTION_CHANGE = 0,
    FORWARD = 1
  } swingDirectionChange = NO_DIRECTION_CHANGE;

  // Configuration parameters
  float angleProgressGain = DEFAULT_ANGLE_PROGRESS_GAIN;
  float progressGainFactor = DEFAULT_PROGRESS_GAIN_FACTOR;
  float progressDecayRate = DEFAULT_PROGRESS_DECAY_RATE;
  float progressDecayExponent = DEFAULT_PROGRESS_DECAY_EXP;
  float minAngleProgress = DEFAULT_MIN_ANGLE_PROGRESS;
  float resetProgressDelayMs = DEFAULT_RESET_PROGRESS_DELAY_MS;
  float pitchUpdateNoiseThreshold = DEFAULT_PITCH_UPDATE_NOISE_THRESHOLD;

  // Private methods
  void initializeIMU()
  {
    Wire.setClock(400000);
    Wire.begin();
    configureSensorRegisters();
    calibrateIMUSensors();
  }

  void configureSensorRegisters()
  {
    Wire.beginTransmission(MPU_ADDRESS);
    Wire.write(0x6B); // Power management register
    Wire.write(0x00); // Wake up the MPU-6050
    Wire.endTransmission(true);

    Wire.beginTransmission(MPU_ADDRESS);
    Wire.write(0x1C); // acceleration config register
    Wire.write(0x08); // +/- 4g full scale range (default +/- 2g)
    Wire.endTransmission(true);
  }

  void calibrateIMUSensors()
  {
    for (int i = 0; i < CALIBRATION_SAMPLE_SIZE; i++)
    {
      readIMUData();
      accumulateCalibrationOffsets();
    }
    calculateAndApplyCalibrationOffsets();
  }

  void accumulateCalibrationOffsets()
  {
    offset.Acc.X += calculateRollAngle();
    offset.Acc.Y += calculatePitchAngle();
    offset.Gyro.X += state.Gyro.X;
    offset.Gyro.Y += state.Gyro.Y;
    offset.Gyro.Z += state.Gyro.Z;
  }

  void calculateAndApplyCalibrationOffsets()
  {
    offset.Acc.X /= CALIBRATION_SAMPLE_SIZE;
    offset.Acc.Y /= CALIBRATION_SAMPLE_SIZE;
    offset.Gyro.X /= CALIBRATION_SAMPLE_SIZE;
    offset.Gyro.Y /= CALIBRATION_SAMPLE_SIZE;
    offset.Gyro.Z /= CALIBRATION_SAMPLE_SIZE;
  }

  void readIMUData()
  {
    Wire.beginTransmission(MPU_ADDRESS);
    Wire.write(0x3B); // Start with accelerometer data register (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDRESS, 7 * 2, true); // Read 14 registers total, each value is stored in 2 registers

    state.Acc.X = readFloatFromWire() / ACCEL_DIVISION_FACTOR;
    state.Acc.Y = readFloatFromWire() / ACCEL_DIVISION_FACTOR;
    state.Acc.Z = readFloatFromWire() / ACCEL_DIVISION_FACTOR;

    // Skip temperature
    readFloatFromWire();

    state.Gyro.X = readFloatFromWire() / GYRO_DIVISION_FACTOR;
    state.Gyro.Y = readFloatFromWire() / GYRO_DIVISION_FACTOR;
    state.Gyro.Z = readFloatFromWire() / GYRO_DIVISION_FACTOR;
  }

  float readFloatFromWire()
  {
    return (Wire.read() << 8 | Wire.read());
  }

  float calculateRollAngle()
  {
    return (atan(state.Acc.Y / sqrt(pow(state.Acc.X, 2) + pow(state.Acc.Z, 2))) * 180 / PI);
  }

  float calculatePitchAngle()
  {
    return (atan(-1 * state.Acc.X / sqrt(pow(state.Acc.Y, 2) + pow(state.Acc.Z, 2))) * 180 / PI);
  }

  // the kalman input is the gyro rate, the kalman measurement is the acceleratometer angle
  KalmanState updateKalmanFilter(KalmanState kalman, float gyroRate, float accelAngle, float elapsedTimeSeconds)
  {
    static const float GYRO_ERROR_DEGREES_PER_SEC_STD_DEV = 4.0;
    static const float ACCEL_UNCERTAINTY_STD_DEV = 3.0;

    kalman.State += elapsedTimeSeconds * gyroRate;
    kalman.Uncertainty += pow(elapsedTimeSeconds, 2) * pow(GYRO_ERROR_DEGREES_PER_SEC_STD_DEV, 2);

    float kalmanGain = kalman.Uncertainty / (kalman.Uncertainty + pow(ACCEL_UNCERTAINTY_STD_DEV, 2));
    kalman.State += kalmanGain * (accelAngle - kalman.State);
    kalman.Uncertainty *= (1 - kalmanGain);

    return kalman;
  }

  void updateAngles()
  {
    previousTime = currentTime;
    currentTime = millis();
    elapsedTimeSeconds = (currentTime - previousTime) / 1000.0;

    readIMUData();

    // Calculate accelerometer angles with offsets
    accelAngle.X = calculateRollAngle() - offset.Acc.X;
    accelAngle.Y = calculatePitchAngle() - offset.Acc.Y;

    // Correct gyro outputs with the calculated offset values
    state.Gyro.X -= offset.Gyro.X;
    state.Gyro.Y -= offset.Gyro.Y;
    state.Gyro.Z -= offset.Gyro.Z;

    // Apply Kalman filter for pitch and roll (can't use kalman filter for yaw)
    kalmanRoll = updateKalmanFilter(kalmanRoll, state.Gyro.X, accelAngle.X, elapsedTimeSeconds);
    kalmanPitch = updateKalmanFilter(kalmanPitch, state.Gyro.Y, accelAngle.Y, elapsedTimeSeconds);

    roll = kalmanRoll.State;
    pitch = kalmanPitch.State;
    yaw = yaw + state.Gyro.Z * elapsedTimeSeconds;

    maxPitchAngle = max(maxPitchAngle, abs(pitch));
  }

  bool shouldUpdateProgress()
  {
    if (abs(pitch) < minAngleProgress)
      return false;
    if (millis() < resetTimeMs)
      return false;
    return true;
  }

  void updateProgressBar()
  {
    if (!shouldUpdateProgress())
      return;

    float gain = pow(abs(pitch), angleProgressGain) / progressGainFactor;
    float decay = progressDecayRate + pow(1 + progressBarValue, progressDecayExponent);
    float newValue = constrain(progressBarValue + gain - decay, 0.0, 100.0);

    if (progressBarValue < 100 && newValue == 100)
    {
      winTimeMs = millis();
    }

    progressBarValue = newValue;
  }

public:
  SwingMotionTracker()
      : audioController(),
        kalmanRoll{0, 2 * 2}, // Initial state and uncertainty: swing starts in a mostly leveled surface, but allowing for some error (+-2 deg)
        kalmanPitch{0, 2 * 2}
  {
    initializeIMU();
  }

  void configureParameters()
  {
    static float volume = 100;
    static float *configurableParameters[] = {
        &angleProgressGain,
        &progressDecayRate,
        &progressGainFactor,
        &progressDecayExponent,
        &volume,
        &resetProgressDelayMs,
        &minAngleProgress};

    if (Serial.available() == 0)
      return;

    int numParams = sizeof(configurableParameters) / sizeof(configurableParameters[0]);
    float newValue;
    char delimiter;

    for (int i = 0; i < numParams; i++)
    {
      newValue = Serial.parseFloat();
      *configurableParameters[i] = newValue;
      delimiter = Serial.read();
      if (delimiter == ';')
        break;
    }

    while (Serial.available() > 0)
      Serial.read();

    audioController.setVolume(volume);
  }

  void reset()
  {
    progressBarValue = 0;
    maxPitchAngle = 0;
    playedMaxLevel = false;
    resetTimeMs = millis() + resetProgressDelayMs;
    audioController.selectRandomVoice();
  }

  void recalibrateOnIdle()
  {
    // if idle (barely moved) for 30 seconds, recailbrate to remove angle drift errors
    const int IS_IDLE_CHECK_MAX_ANGLE_DELTA = 1.5;
    const int IS_IDLE_CHECK_INTERVAL_MS = 30000;
    static float maxPitchInInterval, minPitchInInterval;
    static long nextSampleTimeMs;

    maxPitchInInterval = max(maxPitchInInterval, pitch);
    minPitchInInterval = min(minPitchInInterval, pitch);

    if (millis() >= nextSampleTimeMs)
    {
      if ((maxPitchInInterval - minPitchInInterval) < IS_IDLE_CHECK_MAX_ANGLE_DELTA)
      {
        calibrateIMUSensors();
        reset();
      }
      nextSampleTimeMs = millis() + IS_IDLE_CHECK_INTERVAL_MS;
      maxPitchInInterval = pitch;
      minPitchInInterval = pitch;
    }
  }

  void processReset()
  {
    if (playedMaxLevel)
    {
      reset();
    }

    if (!isSwinging)
    {
      resetTimeMs = millis() + resetProgressDelayMs;
    }

    // if stuck due to serial error, reset progress
    if (progressBarValue == 100 && (millis() - winTimeMs) > 5000)
    {
      reset();
    }

    recalibrateOnIdle();
  }

  void displayState()
  {
    utils::displayFloat("θ", pitch);
    utils::displayFloat("L", level * 10);
    utils::displayFloat("💲", progressBarValue);
    utils::displayFloat("🔁", ((int)swingDirectionChange) * 30, true);
  }

  bool shouldPlaySound()
  {
    if (millis() < resetTimeMs)
      return false;
    if ((level == 0) && (abs(pitch) < minAngleProgress))
      return false;
    if (swingDirectionChange != FORWARD)
      return false;
    return true;
  }

  void handleSound()
  {
    level = floor(map(progressBarValue, 0, 100, 0, audioController.getNumLevels() - 1));

    if (!shouldPlaySound())
      return;
    playedMaxLevel = audioController.tryPlaySound(level);
  }

  // use angle derivative changes to identify swing direction changes
  void updateSwingDirection()
  {
    const float DERIVATIVE_CALC_TIME_WINDOW_MS = 100.0;
    const float D_PITCH_UPDATE_NOISE_THRESHOLD = 0.015;
    static float previousDerivativeUpdateTimeMs = 0;
    static float previousWindowPitchAvg = 0;
    static float pitchSampleSum = 0;
    static float pitchSampleCount = 0;
    static float dPitch, previousDPitch, previousPitch;

    float currentTimeMs = millis();
    swingDirectionChange = NO_DIRECTION_CHANGE;
    isSwinging = maxPitchAngle >= minAngleProgress;

    // calculate the pitch angle derivative using the time average value in the current time window vs the previous time window and the time delta between the windows.
    if (currentTimeMs - previousDerivativeUpdateTimeMs > DERIVATIVE_CALC_TIME_WINDOW_MS)
    {
      float dt = currentTimeMs - previousDerivativeUpdateTimeMs;
      previousDerivativeUpdateTimeMs = currentTimeMs;
      pitchSampleCount = (pitchSampleCount == 0) ? 1 : pitchSampleCount; // 0 div protection
      float currentWindowPitchAvg = pitchSampleSum / pitchSampleCount;
      float newDPitch = (currentWindowPitchAvg - previousWindowPitchAvg) / dt;

      // only update angle derivative above a defined threshold to reduce noise in change direction calculations
      bool shouldUpdateDPitch = abs(newDPitch) > D_PITCH_UPDATE_NOISE_THRESHOLD;
      if (shouldUpdateDPitch)
      {
        dPitch = newDPitch;
      }

      previousWindowPitchAvg = currentWindowPitchAvg;

      if (previousDPitch < 0 && dPitch >= 0)
      {
        swingDirectionChange = FORWARD;
      }
      if (previousDPitch > 0 && dPitch <= 0)
      {
        swingDirectionChange = BACKWARDS;
      }

      pitchSampleSum = 0;
      pitchSampleCount = 0;

      if (shouldUpdateDPitch)
      {
        previousDPitch = dPitch;
      }
    }

    pitchSampleSum += pitch;
    pitchSampleCount++;
  }

  void update()
  {
    configureParameters();
    updateAngles();
    updateSwingDirection();
    updateProgressBar();
    handleSound();
    displayState();
    processReset();
  }
};

SwingMotionTracker *swingTracker = nullptr;

void setup()
{
  Serial.begin(9600);
  swingTracker = new SwingMotionTracker();
}

void loop()
{
  if (swingTracker)
    swingTracker->update();
}