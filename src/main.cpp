#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <SPI.h>
#include <utility/imumaths.h>
#include <EEPROM.h>
#include <math.h>
#include <vector.h>
#include <ArduinoEigen.h>

/*                                            */
/*          Constants and Globals             */
/*                                            */

Adafruit_BNO055 bno = Adafruit_BNO055(55);
sensor_t sensor;

int eeprom_address;
long bno_id;

unsigned long lastSend = 0;
unsigned long start_time;
uint16_t DECIMALS = 4;


/*                                            */
/*             Function Prototypes            */
/*                                            */

bool calibrate(bool);
void print_calibration(uint8_t, uint8_t, uint8_t);

void get_eeprom();
void put_eeprom();

void imu_to_eigen(const imu::Quaternion&, Eigen::Quaterniond&);
imu::Quaternion quaternion_conjugate(imu::Quaternion&);
imu::Quaternion quaternion_normalize(imu::Quaternion&);
imu::Quaternion quaternion_inverse(imu::Quaternion&);
imu::Quaternion quaternion_multiply(const imu::Quaternion&, const imu::Quaternion&);

double quaternion_norm(imu::Quaternion&);
std::vector <double> quat_to_euler(const imu::Quaternion&);

void setup() {

    Serial.begin(115200);
    while (!Serial) {
        delay(10);
    }

    // attempt to connect every 2 seconds until connection successful
    while (!bno.begin()) {
        Serial.println("No BNO055 detected");
        delay(2000);
    }

    bno.setMode(OPERATION_MODE_NDOF);

    // load in offsets from EEPROM if stored
    get_eeprom();

    // keep checking calibration levels every 2 seconds until fully calibrated
    while (!calibrate(true)) {
        delay(2000);
    }

    // load in fresher offsets into EEPROM memory
    put_eeprom();

    bno.setExtCrystalUse(true);
    start_time = millis();
}


void loop() {
    while (!calibrate(false)) {
        if (millis() - lastSend > 20) {

            lastSend = millis();

            imu::Quaternion imu_quat = bno.getQuat();
            Eigen::Quaterniond eigen_quat;
            imu_to_eigen(imu_quat, eigen_quat);
            eigen_quat.normalize();

            auto euler = eigen_quat.toRotationMatrix().eulerAngles(2, 1, 0);

            double roll = euler[2];
            double pitch = euler[1];
            double yaw = euler[0];

            String message = "$";
            message += String(millis() - start_time) + ": ";
            message += String(roll * 180 / M_PI, DECIMALS) + ",";
            message += String(pitch * 180 / M_PI, DECIMALS) + ",";
            message += String(yaw * 180 / M_PI, DECIMALS);

            Serial.println(message);
        }
    }

    

    delay(10);
}


// checks if sensors are fully calibrated
bool calibrate(bool printflag) {
    uint8_t system_cal, gyro_cal, accel_cal, mag_cal;
    bno.getCalibration(&system_cal, &gyro_cal, &accel_cal, &mag_cal);

    // print calibration levels if wanted
    if (printflag) {
        print_calibration(gyro_cal, accel_cal, mag_cal);
    }

    // successfully calibrated if system and sensors are all calibrated fully ( =3 )
    return ( gyro_cal + accel_cal + mag_cal == 9 );
}


// print calibration levels of everything
void print_calibration(uint8_t gyro_cal, uint8_t accel_cal, uint8_t mag_cal) {
    String calibration = "";

    calibration += "Calibrating... ";
    calibration += "Gyro: "  ;    calibration += String(gyro_cal);
    calibration += ", Accel: " ;    calibration += String(accel_cal);
    calibration += ", Magnet: ";    calibration += String(mag_cal);
    Serial.println(calibration);
}


// fetches the stored offsets from EEPROM
void get_eeprom() {
    adafruit_bno055_offsets_t bno_offsets;
    EEPROM.get(eeprom_address, bno_id);
    bno.getSensor(&sensor);

    // no previous offsets stored
    if (bno_id != sensor.sensor_id) {
        return;
    }

    eeprom_address += sizeof(long);
    EEPROM.get(eeprom_address, bno_offsets);
    bno.setSensorOffsets(bno_offsets);

    return;
}


// stores new offsets in EEPROM
void put_eeprom() {
    adafruit_bno055_offsets_t new_offsets;
    bno.getSensorOffsets(new_offsets);

    eeprom_address = 0;
    bno.getSensor(&sensor);
    bno_id = sensor.sensor_id;

    EEPROM.put(eeprom_address, bno_id);

    eeprom_address += sizeof(long);
    EEPROM.put(eeprom_address, new_offsets);
}


// converts Quaternion object of imu library to that of Eigen library
void imu_to_eigen(const imu::Quaternion& q1, Eigen::Quaterniond& q2) {
    q2.w() = q1.w();
    q2.x() = q1.x();
    q2.y() = q1.y();
    q2.z() = q1.z();
}


imu::Quaternion quaternion_conjugate(imu::Quaternion& q) {
    return imu::Quaternion(q.w(), -q.x(), -q.y(), -q.z());
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


double quaternion_norm(imu::Quaternion& q) {
    return sqrt(q.w() * q.w() + q.x() * q.x() + q.y() * q.y() + q.z() * q.z());
}


// convert quaternion to standard roll/pitch/yaw angles
std::vector <double> quat_to_euler(const imu::Quaternion& q) {
    std::vector <double> euler_angles(3);

    double sq_w = q.w() * q.w();
    double sq_x = q.x() * q.x();
    double sq_y = q.y() * q.y();
    double sq_z = q.z() * q.z();

    double unit = sq_w + sq_x + sq_y + sq_z;
    double test = q.x() * q.y() + q.z() * q.w();

    // initial conditionals are for catching singularities when converting to Euler
    if (test > 0.49995 * unit) {
        euler_angles[0] = 0;
        euler_angles[1] = 2 * atan2(q.x(), q.w());
        euler_angles[2] = M_PI / 2;
    }
    else if (test < -0.49995 * unit) {
        euler_angles[0] = 0;
        euler_angles[1] = -2 * atan2(q.x(), q.w());
        euler_angles[2] = -M_PI / 2;
    }
    else {
        euler_angles[0] = atan2(2 * (q.w() * q.x() - q.y() * q.z()), sq_w - sq_x + sq_y - sq_z);
        euler_angles[1] = atan2(2 * (q.w() * q.y() - q.x() * q.z()), sq_w + sq_x - sq_y - sq_z);
        euler_angles[2] = asin(2 * test / unit);
    }

    return euler_angles;
}