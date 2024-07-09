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
/*          Constants and Globals             */
/*                                            */

// sensor objects
Adafruit_BNO055 bno = Adafruit_BNO055(55);
sensor_t sensor;

int lastSend = 0;
unsigned long start_time;
uint16_t DECIMALS = 4;


/*                                            */
/*             Function Prototypes            */
/*                                            */
bool calibrate(bool);
void printCalibration(uint8_t, uint8_t, uint8_t);

double quaternion_norm(imu::Quaternion&);
std::vector <double> quat_to_euler(const imu::Quaternion&);

imu::Quaternion quaternion_conjugate(imu::Quaternion&);
imu::Quaternion quaternion_normalize(imu::Quaternion&);
imu::Quaternion quaternion_inverse(imu::Quaternion&);
imu::Quaternion quaternion_multiply(const imu::Quaternion&, const imu::Quaternion&);


void setup() {

    Serial.begin(115200);
    while (!Serial) {
        delay(10);
    }

    // attempt to connect to bno every 2 seconds until connection formed
    while (!bno.begin()) {
        Serial.println("No BNO055 detected");
        delay(2000);
    }

    // keep checking calibration levels every 2 seconds until fully calibrated
    while (!calibrate(true)) {
        delay(2000);
    }

    bno.setExtCrystalUse(true);
    start_time = millis();

}

void loop() {

    if (millis() - lastSend > 20) {
    //ensures that the sensor data is collected and transmitted at a frequency of approximately 20 milliseconds (50hz) and there is a 1-ms delay between each round of data collection (atm).

        lastSend = millis();

        String quaternion, acceleration, angularvel, message = "0";

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

    }

    delay(10);
}

bool calibrate(bool printflag) {
    uint8_t system_cal, gyro_cal, accel_cal, mag_cal;
    bno.getCalibration(&system_cal, &gyro_cal, &accel_cal, &mag_cal);

    // print 
    if (printflag) {
        printCalibration(gyro_cal, accel_cal, mag_cal);
    }

    // successfully calibrated if system and sensors are all calibrated fully ( =3 )
    return ( gyro_cal + accel_cal + mag_cal == 9 );
}

// Print calibration levels of everything
void printCalibration(uint8_t gyro_cal, uint8_t accel_cal, uint8_t mag_cal) {
    String calibration = "";

    calibration += "Calibrating... ";
    calibration += "Gyro: "  ;    calibration += String(gyro_cal);
    calibration += ", Accel: " ;    calibration += String(accel_cal);
    calibration += ", Magnet: ";    calibration += String(mag_cal);
    Serial.println(calibration);
}

double quaternion_norm(imu::Quaternion& q) {
    return sqrt(q.w() * q.w() + q.x() * q.x() + q.y() * q.y() + q.z() * q.z());
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

std::vector <double> quat_to_euler(const imu::Quaternion& q) {
    std::vector <double> euler_angles(3);

    // initial conditionals are for catching singularities when converting to Euler
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