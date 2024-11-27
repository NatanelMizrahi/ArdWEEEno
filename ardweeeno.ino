/*
   Arduino and MPU6050 Accelerometer and Gyroscope Sensor Tutorial
   by Dejan, https://howtomechatronics.com
*/

#include <Wire.h>

void displayFloat(char* label, float value, bool last=false);

typedef struct  {
  float X, Y, Z;
} coords_t;

typedef struct {
  coords_t Acc;
  coords_t Gyro;
  float speed;
} state_t;

state_t state, prevState, offset, angle;

char floatBuffer[12];

float roll, pitch, yaw;
float elapsedTimeSeconds, currentTime, previousTime;
float lastPrintPreviousTime;
float temperature;

const int MPU = 0x68; // MPU6050 I2C address
const int ACCEL_CONFIG_REGISTER = 0x1C;
const int ACCEL_CONFIG_FULL_RANGE_VAL = 0x10; // +/- 8g full scale range (default +/- 2g)
const int GYRO_CONFIG_REGISTER = 0x1B;
const int GYRO_CONFIG_FULL_RANGE_VAL = 0x10; // 1000deg/s full scale (default +/- 250deg/s)

const float ACCEL_DIVISION_FACTOR = 16384.0; // For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
const float GYRO_DIVISION_FACTOR = 131.0;   // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet

const bool SHOULD_USE_ACCEL_FULL_RANGE = false;
const bool SHOULD_USE_GYRO_FULL_RANGE = false;

const int OFFSET_CALC_SAMPLE_SIZE = 200;
const int DISPLAY_INTERVAL_MS = 500;
const bool SERIAL_PLOTTER_ENABLED = false;

void configureSensitivity() {
  if (SHOULD_USE_ACCEL_FULL_RANGE) {
    Wire.beginTransmission(MPU);
    Wire.write(ACCEL_CONFIG_REGISTER);
    Wire.write(ACCEL_CONFIG_FULL_RANGE_VAL);
    Wire.endTransmission(true);
  }

  if (SHOULD_USE_GYRO_FULL_RANGE) {
    Wire.beginTransmission(MPU);
    Wire.write(GYRO_CONFIG_REGISTER);
    Wire.write(GYRO_CONFIG_FULL_RANGE_VAL);
    Wire.endTransmission(true);
  }

  delay(20);
}

void configureIMU() {
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission

  configureSensitivity();
  calculateImuInitialValuesOffset();
}
void displayFloat(char* label, float value, bool last) {
  if (SERIAL_PLOTTER_ENABLED) {
    Serial.print(label);
    Serial.print(":");
    Serial.print(value);
    if (last)
      Serial.println();
    else
      Serial.print(",");
  }
  else {
    dtostrf(value, 6, 2, floatBuffer);
    Serial.print(" | ");
    Serial.print(label);
    Serial.print(": ");
    Serial.print(floatBuffer);
    if (last)
      Serial.println();
  }
}

void displayState() { 
  if (SERIAL_PLOTTER_ENABLED ||  millis() - lastPrintPreviousTime > DISPLAY_INTERVAL_MS) {
    lastPrintPreviousTime = millis();
    displayFloat("r", roll);
    displayFloat("p", pitch);
    displayFloat("y", yaw);
    displayMeasurements(state);
  }
}

void setup() {
	Serial.begin(19200);
  configureIMU();
  delay(20);
}

float readFloat() {
  return Wire.read() << 8 | Wire.read();
}

void readImuData() {
  prevState = state;

  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 7*2, true); // Read 14 registers total, each value is stored in 2 registers
  
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

void loop() {
  previousTime = currentTime;
  currentTime = millis();
  elapsedTimeSeconds = (currentTime - previousTime) / 1000;

  readImuData();
  
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

  // combine acceleromter and gyro angle values
  yaw =  yaw + state.Gyro.Z * elapsedTimeSeconds;
  roll = 0.96 * angle.Gyro.X + 0.04 * angle.Acc.X;
  pitch = 0.96 * angle.Gyro.Y + 0.04 * angle.Acc.Y;
  
  displayState();
}

void displayMeasurements(state_t s) {
    displayFloat("aX", s.Acc.X);
    displayFloat("aY", s.Acc.Y);
    displayFloat("aZ", s.Acc.Z);
    displayFloat("gX", s.Gyro.X);
    displayFloat("gY", s.Gyro.Y);
    displayFloat("gZ", s.Gyro.Z);
    displayFloat("v", s.speed, true);
}

void calculateImuInitialValuesOffset() {
  // Note that we should place the IMU flat in order to get the correct values.
  
  // Read accelerometer values OFFSET_CALC_SAMPLE_SIZE times
	for (int i = 0; i < OFFSET_CALC_SAMPLE_SIZE; i++) {
		readImuData();
  
    // Sum all readings
		offset.Acc.X = offset.Acc.X + ((atan((state.Acc.Y) / sqrt(pow((state.Acc.X), 2) + pow((state.Acc.Z), 2))) * 180 / PI));
		offset.Acc.Y = offset.Acc.Y + ((atan(-1 * (state.Acc.X) / sqrt(pow((state.Acc.Y), 2) + pow((state.Acc.Z), 2))) * 180 / PI));
  
    offset.Gyro.X = offset.Gyro.X + state.Gyro.X;
		offset.Gyro.Y = offset.Gyro.Y + state.Gyro.Y;
		offset.Gyro.Z = offset.Gyro.Z + state.Gyro.Z;
  }

  // Calculate the average to get the offset value
	offset.Acc.X = offset.Acc.X / OFFSET_CALC_SAMPLE_SIZE;
	offset.Acc.Y = offset.Acc.Y / OFFSET_CALC_SAMPLE_SIZE;

	offset.Gyro.X = offset.Gyro.X / OFFSET_CALC_SAMPLE_SIZE;
	offset.Gyro.Y = offset.Gyro.Y / OFFSET_CALC_SAMPLE_SIZE;
	offset.Gyro.Z = offset.Gyro.Z / OFFSET_CALC_SAMPLE_SIZE;
  
  if (!SERIAL_PLOTTER_ENABLED) {
    Serial.println("******* OFFSET VALUES *******");
    displayMeasurements(offset);
    Serial.println("****************************");
  }
}

// TODO: ignore very small changes
