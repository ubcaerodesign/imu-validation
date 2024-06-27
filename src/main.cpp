#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <SPI.h>
#include <utility/imumaths.h>
#include <EEPROM.h>

/*                                            */
/* Sensor Objects and variables               */
/*                                            */
Adafruit_BNO055 bno = Adafruit_BNO055(55); // i2c address: 0x29                        // i2c address: 0x42

/*                                            */
/* Function Declaration                       */
/*                                            */
String get_bno055(sensors_event_t* event);
void zeroQuat();
double quaternion_norm(imu::Quaternion& q);
imu::Quaternion quaternion_conjugate(imu::Quaternion& q);
imu::Quaternion quaternion_normalize(imu::Quaternion& q);
imu::Quaternion quaternion_multiply(const imu::Quaternion& q1, const imu::Quaternion& q2);

/*                                            */
/* Constants and Globals                      */
/*                                            */
int lastSend = 0;

/* for bno055 */
uint16_t DECIMALS = 4;
imu::Quaternion offset_norm;
sensor_t sensor;

void setup() {

  // Wire.begin();
  Serial.begin(115200);
  while (!Serial) delay(10);

  if (!bno.begin())
  {
    Serial.println("No BNO055 detected");
    while(1);
  }

  Serial.println("BNO055 CONNECTED!!!!");
  bno.setExtCrystalUse(true);

}

void loop() {

  if (millis() - lastSend > 20) { //ensures that the sensor data is collected and transmitted at a frequency of approximately 20 milliseconds (50hz) and there is a 1-ms delay between each round of data collection (atm).

    lastSend = millis();

    String quaternion   ,
           acceleration ,
           angularvel   
           = "0";

    String message = "0";

    sensors_event_t angVelocityData , AccelData;

  // Update all sensors and make them strings to send ...........................................              

    imu::Quaternion quatData = bno.getQuat();
    bno.getEvent(&angVelocityData   , Adafruit_BNO055::VECTOR_GYROSCOPE); // rad/s
    bno.getEvent(&AccelData   , Adafruit_BNO055::VECTOR_ACCELEROMETER);   // m/s^2


    message += "{"  ;
    message += "\"q3\": "     ; message += String(quatData.w(), DECIMALS); message += ",";
    message += "\"q0\": "     ; message += String(quatData.x(), DECIMALS); message += ",";
    message += "\"q1\": "     ; message += String(quatData.y(), DECIMALS); message += ",";
    message += "\"q2\": "     ; message += String(quatData.z()*(-1), DECIMALS); message += ",";
    message += "\"accel_x\": "; message += String(AccelData.acceleration.x, DECIMALS); message += ",";
    message += "\"accel_y\": "; message += String(AccelData.acceleration.y, DECIMALS); message += ",";
    message += "\"accel_z\": "; message += String(AccelData.acceleration.z*(-1), DECIMALS); message += ",";
    message += "\"gyro_x\": " ; message += String(angVelocityData.gyro.x, DECIMALS); message += ",";
    message += "\"gyro_y\": " ; message += String(angVelocityData.gyro.y, DECIMALS); message += ",";
    message += "\"gyro_z\": " ; message += String(angVelocityData.gyro.z*(-1), DECIMALS); message += ",";
    message += "\"time\": "   ; message += lastSend     ;
    message += "}";

    Serial.println(message);


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

imu::Quaternion quaternion_multiply(const imu::Quaternion& q1, const imu::Quaternion& q2) {
  return imu::Quaternion(
    q1.w() * q2.w() - q1.x() * q2.x() - q1.y() * q2.y() - q1.z() * q2.z(),
    q1.w() * q2.x() + q1.x() * q2.w() + q1.y() * q2.z() - q1.z() * q2.y(),
    q1.w() * q2.y() - q1.x() * q2.z() + q1.y() * q2.w() + q1.z() * q2.x(),
    q1.w() * q2.z() + q1.x() * q2.y() - q1.y() * q2.x() + q1.z() * q2.w()
  );
}


// #include <Arduino.h>

// #define ledPin PC13 //13

// void setup() {
//   pinMode(ledPin,OUTPUT);
//   Serial.begin(115200);
// }

// void loop() {
//   digitalWrite(ledPin, HIGH);
//   delay(2000);
//   digitalWrite(ledPin, LOW);
//   delay(2000);
//   Serial.println("Hello!");
// }