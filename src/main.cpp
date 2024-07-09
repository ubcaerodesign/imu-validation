#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <SPI.h>
#include <utility/imumaths.h>
#include <EEPROM.h>
#include <math.h>
#include <vector.h>

/*                                            */
/* Sensor Objects and variables               */
/*                                            */
Adafruit_BNO055 bno = Adafruit_BNO055(55); // i2c address: 0x29                        // i2c address: 0x42

/*                                            */
/* Function Declaration                       */
/*                                            */
String get_bno055(sensors_event_t* event);
bool calibrate();

// Quaternion functions
void zeroQuat();
double quaternion_norm(imu::Quaternion& q);
imu::Quaternion quaternion_conjugate(imu::Quaternion& q);
imu::Quaternion quaternion_normalize(imu::Quaternion& q);
imu::Quaternion quaternion_multiply(const imu::Quaternion& q1, const imu::Quaternion& q2);
imu::Quaternion quaternion_inverse(imu::Quaternion& q);
std::vector <double> quat_to_euler(const imu::Quaternion& q);

/*                                            */
/* Constants and Globals                      */
/*                                            */
int lastSend = 0;
unsigned long start_time;

/* for bno055 */
uint16_t DECIMALS = 4;
imu::Quaternion offset_norm;
sensor_t sensor;

imu::Quaternion reference_quat;
imu::Quaternion inverse_quat;

void setup() {
  
  // Wire.begin();
  Serial.begin(115200);
  while (!Serial) delay(10);

  // attempt to connect to bno every 2 seconds until connection formed
  while (!bno.begin())
  {
    Serial.println("No BNO055 detected");
    delay(2000);
  }

  // keep checking calibration levels every 2 seconds until fully calibrated
  while (!calibrate()) {
    delay(2000);
  }

  bno.setExtCrystalUse(true);
  reference_quat = bno.getQuat();
  inverse_quat = quaternion_inverse(reference_quat);
  start_time = millis();

}

void loop() {

  if (millis() - lastSend > 20) {
    //ensures that the sensor data is collected and transmitted at a frequency of approximately 20 milliseconds (50hz) and there is a 1-ms delay between each round of data collection (atm).

    lastSend = millis();

    String quaternion   ,
           acceleration ,
           angularvel   
           = "0";

    String message = "0";

    sensors_event_t angVelocityData , AccelData;

    imu::Quaternion measured_quat = bno.getQuat();

    // Serial.print(measured_quat.w());
    // Serial.print(measured_quat.x());
    // Serial.print(measured_quat.y());
    // Serial.print(measured_quat.z());
    // Serial.println();

    // imu::Quaternion quat_data = quaternion_multiply(inverse_quat, measured_quat);
    std::vector <double> euler_angles = quat_to_euler(measured_quat);

    double roll = euler_angles[0];
    double pitch = euler_angles[1];
    double yaw = euler_angles[2];

    Serial.println(String(millis() - start_time) + "," + String(roll, DECIMALS));


    // Update all sensors and make them strings to send ...........................................              

    // imu::Quaternion quatData = bno.getQuat();
    // bno.getEvent(&angVelocityData   , Adafruit_BNO055::VECTOR_GYROSCOPE); // rad/s
    // bno.getEvent(&AccelData   , Adafruit_BNO055::VECTOR_ACCELEROMETER);   // m/s^2


    // message += "{"  ;
    // message += "\"q3\": "     ; message += String(quatData.w(), DECIMALS); message += ",";
    // message += "\"q0\": "     ; message += String(quatData.x(), DECIMALS); message += ",";
    // message += "\"q1\": "     ; message += String(quatData.y(), DECIMALS); message += ",";
    // message += "\"q2\": "     ; message += String(quatData.z()*(-1), DECIMALS); message += ",";
    // message += "\"accel_x\": "; message += String(AccelData.acceleration.x, DECIMALS); message += ",";
    // message += "\"accel_y\": "; message += String(AccelData.acceleration.y, DECIMALS); message += ",";
    // message += "\"accel_z\": "; message += String(AccelData.acceleration.z*(-1), DECIMALS); message += ",";
    // message += "\"gyro_x\": " ; message += String(angVelocityData.gyro.x, DECIMALS); message += ",";
    // message += "\"gyro_y\": " ; message += String(angVelocityData.gyro.y, DECIMALS); message += ",";
    // message += "\"gyro_z\": " ; message += String(angVelocityData.gyro.z*(-1), DECIMALS); message += ",";
    // message += "\"time\": "   ; message += lastSend     ;
    // message += "}";

    // Serial.println(message);

  }

  delay(10);
}

String get_bno055(sensors_event_t* event) {
  String w = "err";
  String x = "err";
  String y = "err";
  String z = "err";

  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    x = String(event->acceleration.x, DECIMALS);
    y = String(event->acceleration.y, DECIMALS);
    z = String(event->acceleration.z, DECIMALS);

    return String(x + "," + y + "," + z);
  }

  else if (event->type == SENSOR_TYPE_ORIENTATION) {
    x = String(event->orientation.x, DECIMALS);
    y = String(event->orientation.y, DECIMALS);
    z = String(event->orientation.z, DECIMALS);

    return String(x + "," + y + "," + z);
  }

  else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    x = String(event->magnetic.x, DECIMALS);
    y = String(event->magnetic.y, DECIMALS);
    z = String(event->magnetic.z, DECIMALS);

    return String(x + "," + y + "," + z);
  }

  else if (event->type == SENSOR_TYPE_GYROSCOPE) {
    x = String(event->gyro.x, DECIMALS);
    y = String(event->gyro.y, DECIMALS);
    z = String(event->gyro.z, DECIMALS);

    return String(x + "," + y + "," + z);
  }
  else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
    x = String(event->gyro.x, DECIMALS);
    y = String(event->gyro.y, DECIMALS);
    z = String(event->gyro.z, DECIMALS);

  }

  else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    x = String(event->acceleration.x, DECIMALS);
    y = String(event->acceleration.y, DECIMALS);
    z = String(event->acceleration.z, DECIMALS);

    return String(x + "," + y + "," + z);
  }

  else if (event->type == SENSOR_TYPE_GRAVITY) {
    x = String(event->acceleration.x, DECIMALS);
    y = String(event->acceleration.y, DECIMALS);
    z = String(event->acceleration.z, DECIMALS);

    return String(x + "," + y + "," + z);
  }

  return "err";
}

bool calibrate() {
  String calibration = "";
  uint8_t system_cal, gyro_cal, accel_cal, mag_cal;
  bno.getCalibration(&system_cal, &gyro_cal, &accel_cal, &mag_cal);

  // print calibration levels of everything
  calibration += "Calibrating... ";
  calibration += "System: "  ;    calibration += String(system_cal);
  calibration += ", Gyro: "  ;    calibration += String(gyro_cal);
  calibration += ", Accel: " ;    calibration += String(accel_cal);
  calibration += ", Magnet: ";    calibration += String(mag_cal);
  Serial.println(calibration);

  // successfully calibrated if system and sensors are all calibrated fully ( =3 )
  return ( system_cal + gyro_cal + accel_cal + mag_cal == 12 );
}

void zeroQuat() {
    imu::Quaternion offset = bno.getQuat();
    imu::Quaternion offset_conj = quaternion_conjugate(offset);
    offset_norm = quaternion_normalize(offset_conj);
}

double quaternion_norm(imu::Quaternion& q) {
  return sqrt(q.w() * q.w() + q.x() * q.x() + q.y() * q.y() + q.z() * q.z());
}

imu::Quaternion quaternion_conjugate(imu::Quaternion& q) {
  return imu::Quaternion(q.w(), -q.x(), -q.y(), -q.z()); //IF USING NED, MAKE Z NEGATIVE TOO!
}

imu::Quaternion quaternion_normalize(imu::Quaternion& q) {
  double norm = quaternion_norm(q);
  return imu::Quaternion(q.w() / norm, q.x() / norm, q.y() / norm, q.z() / norm);
}

imu::Quaternion quaternion_inverse(imu::Quaternion& q) {
  imu::Quaternion conjugate = quaternion_conjugate(q);
  return quaternion_normalize(conjugate);
}

imu::Quaternion quaternion_multiply(const imu::Quaternion& q1, const imu::Quaternion& q2) {
  return imu::Quaternion(
    q1.w() * q2.w() - q1.x() * q2.x() - q1.y() * q2.y() - q1.z() * q2.z(),
    q1.w() * q2.x() + q1.x() * q2.w() + q1.y() * q2.z() - q1.z() * q2.y(),
    q1.w() * q2.y() - q1.x() * q2.z() + q1.y() * q2.w() + q1.z() * q2.x(),
    q1.w() * q2.z() + q1.x() * q2.y() - q1.y() * q2.x() + q1.z() * q2.w()
  );
}

std::vector <double> quat_to_euler(const imu::Quaternion& q) {
  std::vector <double> euler_angles(3);

  // two initial conditionals are for catching gimbal lock while converting to Euler
  if (abs(q.x() * q.y() + q.z() * q.w() - 0.5) < 0.01) {
    euler_angles[0] = 2 * atan2(q.x(), q.w());
    euler_angles[1] = M_PI / 2;
    euler_angles[2] = 0;
  }
  else if (abs(q.x() * q.y() + q.z() * q.w() + 0.5) < 0.01) {
    euler_angles[0] = -2 * atan2(q.x(), q.w());
    euler_angles[1] = -M_PI / 2;
    euler_angles[2] = 0;
  }
  else {
    euler_angles[0] = atan2(2 * (q.w() * q.x() + q.y() * q.z()), 1 - 2 * (q.x() * q.x() + q.y() * q.y()));
    euler_angles[1] = asin(2 * (q.w() * q.y() - q.x() * q.z()));
    euler_angles[2] = atan2(2 * (q.w() * q.z() + q.x() * q.y()), 1 - 2 * (q.y() * q.y() + q.z() * q.z()));
  }

  return euler_angles;
}