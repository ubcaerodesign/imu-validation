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

#define DECIMALS 4
#define ACCEL_SYNC_TIME 5000
#define ACCEL_SYNC_THRESHOLD 0.15

Adafruit_BNO055 bno = Adafruit_BNO055(55);
sensor_t sensor;

int eeprom_address;
long bno_id;

int lastSend = 0;
unsigned long start_time;

imu::Vector <3> g_ref;
imu::Quaternion quat_ref;
imu::Quaternion quat_ref_inverse;
std::vector <std::vector <double>> rot_matrix {
    {0, 0, 0},
    {0, 0, 0},
    {0, 0, 0},
};



/*                                            */
/*             Function Prototypes            */
/*                                            */

bool calibrate(bool);
void print_calibration(uint8_t, uint8_t, uint8_t);

void get_eeprom();
void put_eeprom();
void get_ref();

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

    // get reference gravity vector and quaternion
    get_ref();

    bno.setExtCrystalUse(true);
    start_time = millis();
}


void loop() {
    while (!calibrate(false)) {
        if (millis() - lastSend > 20) {

            lastSend = millis();

            // token to discern data when grabbing data through  Python
            String message = "$,";

            // quaternion to euler data
            imu::Quaternion measured_quat = bno.getQuat();
            std::vector <double> euler_angles = quat_to_euler(measured_quat);

            double roll = euler_angles[0];
            double pitch = euler_angles[1];
            double yaw = euler_angles[2];

            // accelerometer data
            imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

            // magnetometer data
            imu::Quaternion quat_diff = quaternion_multiply(quat_ref_inverse, measured_quat);



            message += String(millis() - start_time)        + ",";
            message += String(roll * 180 / M_PI, DECIMALS)  + ",";
            message += String(pitch * 180 / M_PI, DECIMALS) + ",";
            message += String(yaw * 180 / M_PI, DECIMALS)   + ",";
            message += String(accel.x(), DECIMALS)          + ",";
            message += String(accel.y(), DECIMALS)          + ",";
            message += String(accel.z(), DECIMALS)          + ",";

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

void get_ref() {
    // initial test variables
    imu::Vector <3> initial_acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    imu::Vector <3> new_acc;
    unsigned long start_test = millis();

    // checks if IMU is stationary within defined test duration
    while ( millis() - start_test <= ACCEL_SYNC_TIME ) {
        new_acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

        // checks if acceleration components exceed defined threshold
        if ( !( abs(new_acc.x() - initial_acc.x()) < ACCEL_SYNC_THRESHOLD &&
                abs(new_acc.y() - initial_acc.y()) < ACCEL_SYNC_THRESHOLD &&
                abs(new_acc.z() - initial_acc.z()) < ACCEL_SYNC_THRESHOLD  )) {

            // update start time and initial acceleration
            initial_acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
            start_test = millis();
        }
    }

    // get reference gravity vector and quaternion
    g_ref = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    quat_ref = bno.getQuat();
    quat_ref_inverse = quaternion_inverse(quat_ref);

    return;
}

imu::Quaternion quaternion_inverse(imu::Quaternion& q) {
    return imu::Quaternion(q.w(), -q.x(), -q.y(), -q.z());
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

    euler_angles[0] = atan2(2 * (q.w() * q.x() + q.y() * q.z()), sq_y + sq_w - sq_x - sq_z);
    euler_angles[1] = atan2(2 * (q.w() * q.y() - q.x() * q.z()), sq_w + sq_x - sq_y - sq_z);

    // initial conditionals are for catching singularities when converting to Euler
    if (test > 0.4999 * unit) {
        euler_angles[2] = M_PI / 2;
    }
    else if (test < -0.4999 * unit) {
        euler_angles[2] = -M_PI / 2;
    }
    else {
        euler_angles[2] = asin(2 * test / unit);
    }

    return euler_angles;
}