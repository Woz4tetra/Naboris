
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Adafruit_NeoPixel.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

#include <Atlasbuggy.h>


Atlasbuggy robot("hardware_interface");

/* ----------------------- *
 * BNO055 global variables *
 * ----------------------- */

#define INCLUDE_FILTERED_DATA
// #define INCLUDE_MAG_DATA
// #define INCLUDE_GYRO_DATA
// #define INCLUDE_ACCEL_DATA
#define INCLUDE_LINACCEL_DATA

Adafruit_BNO055 bno = Adafruit_BNO055();

imu::Quaternion quat;
imu::Vector<3> euler;
imu::Vector<3> mag;
imu::Vector<3> gyro;
imu::Vector<3> accel;
imu::Vector<3> linaccel;

// Accelerometer & gyroscope only for getting relative orientation, subject to gyro drift
// Adafruit_BNO055 bno = Adafruit_BNO055(0x08); // OPERATION_MODE_IMUPLUS

// Accelerometer & magnetometer only for getting relative orientation
// Adafruit_BNO055 bno = Adafruit_BNO055(0x0a);  // OPERATION_MODE_M4G

// Gets heading only from compass
// Adafruit_BNO055 bno = Adafruit_BNO055(0x09); // OPERATION_MODE_COMPASS

// OPERATION_MODE_NDOF without fast magnetometer calibration
// Adafruit_BNO055 bno = Adafruit_BNO055(OPERATION_MODE_NDOF_FMC_OFF);

/* ----------------------------- *
 * Motor shield global variables *
 * ----------------------------- */

// #define ENABLE_MOTOR_TIMEOUT_PINGS

 // Create the motor shield object with the default I2C address
 Adafruit_MotorShield AFMS = Adafruit_MotorShield();
 // Or, create it with a different I2C address (say for stacking)
 // Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61);

unsigned int speed_increment = 40;

 struct MotorStruct {
     Adafruit_DCMotor* af_motor;
     int speed;
     int goal_speed;
     byte run_state;
 };

 MotorStruct init_motor(int motor_num) {
     MotorStruct new_motor;
     new_motor.af_motor = AFMS.getMotor(motor_num);
     new_motor.speed = 0;
     new_motor.goal_speed = 0;
     return new_motor;
 }

#define NUM_MOTORS 4
MotorStruct* motors = new MotorStruct[NUM_MOTORS];


#define MOTOR1 1
#define MOTOR2 0
#define MOTOR3 2
#define MOTOR4 3

/* ------------------------ *
 * Encoder global variables *
 * ------------------------ */

Encoder rightEncoder(2, 8);
Encoder leftEncoder(3, 12);

long oldLeftPosition = 0;
long newRightPosition = 0;
long oldRightPosition = 0;
long newLeftPosition = 0;
uint32_t prev_enc_time = 0;

#define R_ENC_BUF_LEN 16
#define L_ENC_BUF_LEN 16
char r_enc_print_buffer[R_ENC_BUF_LEN];
char l_enc_print_buffer[L_ENC_BUF_LEN];

/* ---------------------- *
 * Servo global variables *
 * ---------------------- */

#define SERVO_PIN 9
Servo head_servo;

/* -------------------------- *
 * LED strip global variables *
 * -------------------------- */

#define NUM_LEDS 24
#define LED_SIGNAL_PIN 6

#define SIGNAL_DELAY 1
#define SIGNAL_INCREMENT 1
#define SIGNAL_CYCLES 4
#define SIGNAL_COLOR 100

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, LED_SIGNAL_PIN, NEO_GRB + NEO_KHZ800);
int signal_r, signal_g, signal_b = 0;

/* ------------------------ *
 * General global variables *
 * ------------------------ */

#ifdef ENABLE_MOTOR_TIMEOUT_PINGS
uint32_t ping_timer = millis();
#endif

#define INIT_DATA_BUF_SIZE 255

void setup() {
    robot.begin();

    AFMS.begin();
    strip.begin();

    if(!bno.begin())
    {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.print("Oops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while(1);
    }
    delay(1000);
    bno.setExtCrystalUse(true);

    strip.show();

    for (int motor_num = 1; motor_num <= NUM_MOTORS; motor_num++) {
        motors[motor_num - 1] = init_motor(motor_num);
    }

    char init_data_buf[INIT_DATA_BUF_SIZE];
    snprintf(init_data_buf, INIT_DATA_BUF_SIZE, "%d\t%d", bno.getTemp(), NUM_LEDS);
    robot.setInitData(init_data_buf);

    // fadeColors(0, 0, 0, SIGNAL_COLOR, SIGNAL_COLOR, SIGNAL_COLOR, 1, SIGNAL_DELAY, SIGNAL_INCREMENT);
    fadeColors(0, 0, 0, 0, SIGNAL_COLOR, 0, SIGNAL_CYCLES, SIGNAL_DELAY, SIGNAL_INCREMENT);
    // fadeColors(SIGNAL_COLOR, 0, SIGNAL_COLOR, 0, 0, 0, 1, SIGNAL_DELAY, SIGNAL_INCREMENT);
}

float qw, qx, qy, qz;
float ex, ey, ez;
float mx, my, mz;
float gx, gy, gz;
float ax, ay, az;
float lx, ly, lz;
uint8_t sys_stat, gyro_stat, accel_stat, mag_stat = 0;

#ifdef INCLUDE_FILTERED_DATA
uint16_t imu_skip_counter = 0;
#endif

void updateIMU() {
    // Possible vector values can be:
    // - VECTOR_ACCELEROMETER - m/s^2
    // - VECTOR_MAGNETOMETER  - uT
    // - VECTOR_GYROSCOPE     - rad/s
    // - VECTOR_EULER         - degrees
    // - VECTOR_LINEARACCEL   - m/s^2
    // - VECTOR_GRAVITY       - m/s^2

    Serial.print("imu\tt");
    Serial.print(millis());

    #ifdef INCLUDE_FILTERED_DATA
    // Quaternion data
    imu::Quaternion quat = bno.getQuat();

    float new_qw = quat.w();
    float new_qx = quat.x();
    float new_qy = quat.y();
    float new_qz = quat.z();

    if (new_qw != qw) {
        Serial.print("\tqw");
        Serial.print(qw, 4);
        qw = new_qw;
    }

    if (new_qx != qx) {
        Serial.print("\tqx");
        Serial.print(qx, 4);
        qx = new_qx;
    }

    if (new_qy != qy) {
        Serial.print("\tqy");
        Serial.print(qy, 4);
        qy = new_qy;
    }

    if (new_qz != qz) {
        Serial.print("\tqz");
        Serial.print(qz, 4);
        qz = new_qz;
    }

    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

    float new_ex = euler.x();
    float new_ey = euler.y();
    float new_ez = euler.z();

    // xyz is yaw pitch roll. switching roll pitch yaw
    if (new_ex != ex) {
        Serial.print("\tez");
        Serial.print(ex, 4);
        ex = new_ex;
    }

    if (new_ey != ey) {
        Serial.print("\tey");
        Serial.print(ey, 4);
        ey = new_ey;
    }

    if (new_ez != ez) {
        Serial.print("\tex");
        Serial.print(ez, 4);
        ez = new_ez;
    }
    #endif

    #ifdef INCLUDE_MAG_DATA
    imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

    float new_mx = mag.x();
    float new_my = mag.y();
    float new_mz = mag.z();

    if (new_mx != mx) {
        Serial.print("\tmx");
        Serial.print(mx, 4);
        mx = new_mx;
    }

    if (new_my != my) {
        Serial.print("\tmy");
        Serial.print(my, 4);
        my = new_my;
    }

    if (new_mz != mz) {
        Serial.print("\tmz");
        Serial.print(mz, 4);
        mz = new_mz;
    }
    #endif

    #ifdef INCLUDE_GYRO_DATA
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

    float new_gx = gyro.x();
    float new_gy = gyro.y();
    float new_gz = gyro.z();

    if (new_gx != gx) {
        Serial.print("\tgx");
        Serial.print(gx, 4);
        gx = new_gx;
    }

    if (new_gy != gy) {
        Serial.print("\tgy");
        Serial.print(gy, 4);
        gy = new_gy;
    }

    if (new_gz != gz) {
        Serial.print("\tgz");
        Serial.print(gz, 4);
        gz = new_gz;
    }
    #endif

    #ifdef INCLUDE_ACCEL_DATA
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

    float new_ax = accel.x();
    float new_ay = accel.y();
    float new_az = accel.z();

    if (new_ax != ax) {
        Serial.print("\tax");
        Serial.print(ax, 4);
        ax = new_ax;
    }

    if (new_ay != ay) {
        Serial.print("\tay");
        Serial.print(ay, 4);
        ay = new_ay;
    }

    if (new_az != az) {
        Serial.print("\taz");
        Serial.print(az, 4);
        az = new_az;
    }
    #endif

    #ifdef INCLUDE_LINACCEL_DATA
    imu::Vector<3> linaccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

    float new_lx = linaccel.x();
    float new_ly = linaccel.y();
    float new_lz = linaccel.z();

    if (new_lx != lx) {
        Serial.print("\tlx");
        Serial.print(lx, 4);
        lx = new_lx;
    }

    if (new_ly != ly) {
        Serial.print("\tly");
        Serial.print(ly, 4);
        ly = new_ly;
    }

    if (new_lz != lz) {
        Serial.print("\tlz");
        Serial.print(lz, 4);
        lz = new_lz;
    }
    #endif

    /* Display calibration status for each sensor. */
    bno.getCalibration(&sys_stat, &gyro_stat, &accel_stat, &mag_stat);
    Serial.print("\tss");
    Serial.print(sys_stat, DEC);
    Serial.print("\tsg");
    Serial.print(gyro_stat, DEC);
    Serial.print("\tsa");
    Serial.print(accel_stat, DEC);
    Serial.print("\tsm");
    Serial.print(mag_stat, DEC);

    Serial.print('\n');

}


void set_motor_speed(int motor_num)
{
    if (motors[motor_num].speed > 0) {
        motors[motor_num].af_motor->run(FORWARD);
    }
    else if (motors[motor_num].speed == 0) {
        motors[motor_num].af_motor->run(BRAKE);
    }
    else {
        motors[motor_num].af_motor->run(BACKWARD);
    }
    motors[motor_num].af_motor->setSpeed(abs(motors[motor_num].speed));
}

void set_motor_goal(int motor_num, int speed) {
    motors[motor_num].goal_speed = speed;
    if (motors[motor_num].goal_speed > 0) {
        if (motors[motor_num].goal_speed < 0) {
            motors[motor_num].goal_speed = 0;
        }
        if (motors[motor_num].goal_speed > 255) {
            motors[motor_num].goal_speed = 255;
        }
    }
    else {
        if (motors[motor_num].goal_speed > 0) {
            motors[motor_num].goal_speed = 0;
        }
        if (motors[motor_num].goal_speed < -255) {
            motors[motor_num].goal_speed = -255;
        }
    }
}

// top left, top right, bottom left, bottom right
void set_motors(int speed1, int speed2, int speed3, int speed4)
{
    set_motor_goal(MOTOR1, speed1);  // top left
    set_motor_goal(MOTOR2, speed2);  // top right
    set_motor_goal(MOTOR3, speed3);  // bottom left
    set_motor_goal(MOTOR4, speed4);  // bottom right
}

#ifdef ENABLE_MOTOR_TIMEOUT_PINGS
void ping() {
    ping_timer = millis();
}
#endif

void stop_motors() {
    set_motors(0, 0, 0, 0);
}

void release_motors()
{
    for (int motor_num = 0; motor_num < NUM_MOTORS; motor_num++)
    {
        motors[motor_num].goal_speed = 0;
        motors[motor_num].speed = 0;
        motors[motor_num].af_motor->run(RELEASE);
    }
}

void updateMotors()
{
    for (int motor_num = 0; motor_num < NUM_MOTORS; motor_num++)
    {
        set_motor_speed(motor_num);

        if (motors[motor_num].speed < motors[motor_num].goal_speed) {
            motors[motor_num].speed += speed_increment;
        }
        else {
            motors[motor_num].speed -= speed_increment;
        }

        if (abs(motors[motor_num].speed - motors[motor_num].goal_speed) < 2 * speed_increment) {
            motors[motor_num].speed = motors[motor_num].goal_speed;
        }
    }
}

void updateEncoders()
{
    uint32_t enc_time = millis();
    newRightPosition = rightEncoder.read();
    newLeftPosition = leftEncoder.read();

    if (newRightPosition != oldRightPosition) {
        // snprintf(r_enc_print_buffer, R_ENC_BUF_LEN, "er%lu\t%li\n", enc_time, newRightPosition);

        oldRightPosition = newRightPosition;
        Serial.print("er");
        Serial.print(enc_time);
        Serial.print('\t');
        Serial.print(newRightPosition);
        Serial.print('\n');
        // Serial.print(r_enc_print_buffer);
    }

    if (newLeftPosition != oldLeftPosition) {
        // snprintf(l_enc_print_buffer, L_ENC_BUF_LEN, "el%lu\t%li\n", enc_time, newLeftPosition);

        oldLeftPosition = newLeftPosition;
        Serial.print("el");
        Serial.print(enc_time);
        Serial.print('\t');
        Serial.print(newLeftPosition);
        Serial.print('\n');
        // Serial.print(l_enc_print_buffer);
    }
}

void loop()
{
    while (robot.available())
    {
        int status = robot.readSerial();
        if (status == 0) {  // user command
            String command = robot.getCommand();
            if (command.charAt(0) == 'd') {  // drive command
                int m1 = command.substring(1, 5).toInt();
                int m2 = command.substring(5, 9).toInt();
                int m3 = command.substring(9, 13).toInt();
                int m4 = command.substring(13, 17).toInt();

                set_motors(m1, m2, m3, m4);
                #ifdef ENABLE_MOTOR_TIMEOUT_PINGS
                ping();
                #endif
            }

            else if (command.charAt(0) == 'h') {  // stop command
                stop_motors();
            }
            else if (command.charAt(0) == 'r') {  // release command
                release_motors();
            }
            else if (command.charAt(0) == 'c') {  // servo command
                int servo_value = command.substring(1, 4).toInt();
                head_servo.write(servo_value);
            }
            else if (command.charAt(0) == 'o') {  // pixel command
                int led_num = command.substring(1, 4).toInt();
                if (led_num < 0) {
                    led_num = 0;
                }
                int r = command.substring(4, 7).toInt();
                int g = command.substring(7, 10).toInt();
                int b = command.substring(10, 13).toInt();
                if (command.length() > 13)
                {
                    int stop_num = command.substring(13, 16).toInt();
                    if (stop_num > NUM_LEDS) {
                        stop_num = NUM_LEDS;
                    }
                    for (int index = led_num; index < stop_num; index++) {
                        strip.setPixelColor(index, strip.Color(r, g, b));
                    }
                }
                else {
                    strip.setPixelColor(led_num, strip.Color(r, g, b));
                }
            }
            else if (command.charAt(0) == 'f' && command.length() == 13) {  // command leds to fade to a color
                int cycle_num = command.substring(1, 4).toInt();
                int r = command.substring(4, 7).toInt();
                int g = command.substring(7, 10).toInt();
                int b = command.substring(10, 13).toInt();
                // fadeColors(SIGNAL_COLOR, SIGNAL_COLOR, SIGNAL_COLOR, 1, SIGNAL_DELAY, SIGNAL_INCREMENT);
                fadeColors(0, 0, 0, r, g, b, cycle_num, SIGNAL_DELAY, SIGNAL_INCREMENT);
            }
            else if (command.charAt(0) == 'x') {  // show command
                head_servo.detach();
                delay(5);
                strip.show();
                delay(5);
                head_servo.attach(SERVO_PIN);
            }
        }
        else if (status == 2) {  // start event
            fadeColors(0, 0, 0, 0, 0, SIGNAL_COLOR, SIGNAL_CYCLES, SIGNAL_DELAY, SIGNAL_INCREMENT);

            stop_motors();
            head_servo.attach(SERVO_PIN);
            rightEncoder.write(0);
            leftEncoder.write(0);
            oldLeftPosition = -1;
            oldRightPosition = -1;
        }
        else if (status == 1) {  // stop event
            stop_motors();
            release_motors();
            head_servo.detach();

            for (int index = 0; index < NUM_LEDS; index++) {
                strip.setPixelColor(index, 0);
            }
            strip.show();

            fadeColors(0, 0, 0, SIGNAL_COLOR, 0, 0, SIGNAL_CYCLES, SIGNAL_DELAY, SIGNAL_INCREMENT);
        }
    }

    if (!robot.isPaused()) {
        #ifdef ENABLE_MOTOR_TIMEOUT_PINGS
        if (ping_timer > millis())  ping_timer = millis();
        if ((millis() - ping_timer) > 500) {
            stop_motors();
            ping_timer = millis();
        }
        #endif

        updateMotors();
        updateEncoders();
        updateIMU();

        // 100Hz update rate for imu
        delay(10);
    }
}

void fadeColors(int r, int g, int b, uint16_t cycles, uint8_t wait, int increment) {
    fadeColors(signal_r, signal_g, signal_b, r, g, b, cycles, wait, increment);
}

void fadeColors(int r1, int g1, int b1, int r2, int g2, int b2, uint16_t cycles, uint8_t wait, int increment)
{
    // Serial.print(r1); Serial.print('\t');
    // Serial.print(g1); Serial.print('\t');
    // Serial.print(b1); Serial.print('\n');
    // Serial.print(r2); Serial.print('\t');
    // Serial.print(g2); Serial.print('\t');
    // Serial.print(b2); Serial.print('\n');

    if (cycles % 2 == 0) {
        signal_r = r1;
        signal_g = g1;
        signal_b = b1;
    }
    else {
        signal_r = r2;
        signal_g = g2;
        signal_b = b2;
    }
    int red_diff = abs(r2 - r1);
    int green_diff = abs(g2 - g1);
    int blue_diff = abs(b2 - b1);

    char max_channel = 'r';
    int max_diff = red_diff;

    if (green_diff > max_diff) {
        max_diff = green_diff;
        max_channel = 'g';
    }
    if (blue_diff > max_diff) {
        max_diff = blue_diff;
        max_channel = 'b';
    }
    // Serial.println(max_channel);

    float red_slope = 0.0;
    float green_slope = 0.0;
    float blue_slope = 0.0;

    int start = 0;
    int end = 0;

    bool condition = true;

    switch (max_channel) {
        case 'r':
            if (r2 < r1) {
                increment *= -1;
            }
            break;
        case 'g':
            if (g2 < g1) {
                increment *= -1;
            }
            break;
        case 'b':
            if (b2 < b1) {
                increment *= -1;
            }
            break;
    }

    // Serial.println(cycles);
    for (uint16_t cycle = 0; cycle < cycles; cycle++)
    {
        switch (max_channel) {
            case 'r':
                condition = r1 < r2;
                if (increment < 0) {
                    condition = !condition;
                }
                if (condition) {
                    start = r1;
                    end = r2;
                }
                else {
                    start = r2;
                    end = r1;
                }
                green_slope = (float)(g2 - g1) / (r2 - r1);
                blue_slope = (float)(b2 - b1) / (r2 - r1);

                if (start < end) {
                    for (int value = start; value <= end; value += increment) {
                        setColor(strip.Color(
                                value,
                                green_slope * (value - r1) + g1,
                                blue_slope * (value - r1) + b1
                            )
                        );
                        delay(wait);
                    }
                }
                else if (start > end) {
                    for (int value = start; value >= end; value += increment) {
                        setColor(strip.Color(
                                value,
                                green_slope * (value - r1) + g1,
                                blue_slope * (value - r1) + b1
                            )
                        );
                        delay(wait);
                    }
                }
                break;

            case 'g':
                condition = g1 < g2;
                if (increment < 0) {
                    condition = !condition;
                }
                if (condition) {
                    start = g1;
                    end = g2;
                }
                else {
                    start = g2;
                    end = g1;
                }


                red_slope = (float)(r2 - r1) / (g2 - g1);
                blue_slope = (float)(b2 - b1) / (g2 - g1);
                if (start < end) {
                    for (int value = start; value <= end; value += increment) {
                        setColor(strip.Color(
                                red_slope * (value - g1) + r1,
                                value,
                                blue_slope * (value - g1) + b1
                            )
                        );
                        delay(wait);
                    }
                }
                else {
                    for (int value = start; value >= end; value += increment) {
                        setColor(strip.Color(
                                red_slope * (value - g1) + r1,
                                value,
                                blue_slope * (value - g1) + b1
                            )
                        );
                        delay(wait);
                    }
                }
                break;
            case 'b':
                condition = b1 < b2;
                if (increment < 0) {
                    condition = !condition;
                }
                if (condition) {
                    start = b1;
                    end = b2;
                }
                else {
                    start = b2;
                    end = b1;
                }
                red_slope = (float)(r2 - r1) / (b2 - b1);
                green_slope = (float)(g2 - g1) / (b2 - b1);

                if (start < end) {
                    for (int value = start; value <= end; value += increment) {
                        setColor(strip.Color(
                                red_slope * (value - b1) + r1,
                                green_slope * (value - b1) + g1,
                                value
                            )
                        );
                        delay(wait);
                    }
                }
                else {
                    for (int value = start; value >= end; value += increment) {
                        setColor(strip.Color(
                                red_slope * (value - b1) + r1,
                                green_slope * (value - b1) + g1,
                                value
                            )
                        );
                        delay(wait);
                    }
                }

                break;
        }
        increment *= -1;

    }
}

void setColor(uint32_t c)
{
    for(uint16_t i = 0; i < strip.numPixels(); i++) {
        strip.setPixelColor(i, c);
    }
    strip.show();
}
