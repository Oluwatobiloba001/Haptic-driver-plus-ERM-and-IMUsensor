#include <Wire.h>               // i2c comms
// #include <Arduino_LSM6DSOX.h>   // wrapper lib for imu. not used
#include <Adafruit_LSM6DSOX.h>  // COOLER wrapper lib??
#include <Adafruit_DRV2605.h>   // wrapper lib for driver



Adafruit_DRV2605 drv;     // create instance of driver
Adafruit_LSM6DSOX imu;    // create instance of imu

// the motors we currently have and the libraries that work best with them:
enum Haptics {
  FIT0774
};

// fancy adafruit unified sensor format datatypes
sensors_event_t accl;
sensors_event_t gyro;
sensors_event_t temp;
float acclNormSquared; // squared norm of the acceleration vector

void setup() {
  // launch I2C
  Wire.setWireTimeout(100000);
  // Wire.begin();

  // launch serial monitor
  while (!Serial);
  Serial.begin(115200);
  Serial.println();
  Serial.println("\n  Serial comms initiated!");

  // try to launch LSM6DSOX
  if (!imu.begin_I2C()) {
    Serial.println("! Failed to initialize the LSM6DSOX IMU :(");
    Serial.println("  Check all connections and press the on-board RESET button to try again");
    while(1); // freeze
  } else {
    Serial.println("  LSM6DSOX IMU initiated!");
    imu.getAccelRange();
    imu.getGyroRange();
    imu.getAccelDataRate();
    imu.getGyroDataRate();
    // setup as per adafruit's example:
    // imu.reset();
    // imu.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
    // imu.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
    // imu.setAccelDataRate(LSM6DS_RATE_104_HZ);
    // imu.setGyroDataRate(LSM6DS_RATE_104_HZ);
    // imu.highPassFilter(0, 0);
  }

  // try to launch DRV2605L
  Serial.println("Initiating all 5 haptic drivers...");
  for (uint8_t i = 0; i < 5; i++) {
    muxSelect(i);
    if (!drv.begin()) {
      Serial.print("! Failed to initialize the DRV2605L haptic driver on channel ");
      Serial.print(i);
      Serial.println(" :(");
      Serial.println("  Check all connections and press the on-board RESET button to try again");
      while(1); // freeze
    } else {
      Serial.print("  DRV2605L haptic driver initiated on channel ");
      Serial.print(i);
      Serial.println("!");
      drv.setMode(0x00);
    }
  }
}

void loop() {

  // Serial.println(muxSelect(0));

  // get readings
  imu.getEvent(&accl, &gyro, &temp);

  acclNormSquared = pow(accl.acceleration.x, 2) + pow(accl.acceleration.y, 2) + pow(accl.acceleration.z, 2);

  // Serial.println(sqrt(acclNormSquared));
  if (acclNormSquared >= 200) {
    runHaptic(3, GS19605, 13);
  }

  Serial.print("\t\tAccel X: ");
  Serial.print(accl.acceleration.x);
  Serial.print(" \tY: ");
  Serial.print(accl.acceleration.y);
  Serial.print(" \tZ: ");
  Serial.print(accl.acceleration.z);
  Serial.println(" m/s^2 ");

  // delay(100);
}

// run haptic motor
void runHaptic(uint8_t driver, Haptics motor, uint8_t effect) {
  if (muxSelect(driver)) {
    Serial.println("! Haptic driver inaccessible :(");
  }
  if (motor == FIT0774) {
    drv.useERM();
    drv.selectLibrary(4);
  } else {
    Serial.println("! Haptic motor isn't supported :(");
    return;
  }

  drv.setWaveform(0, effect);
  drv.setWaveform(1, 0);
  drv.go();
}

