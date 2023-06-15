/**************************************************************************************

File: compass.ino
Author: Tiklon
Date: 14.06.2023
Description: This file contains functions for calibrating sensors, handling sensor input,
             and calculating directions. It is specifically related to the input of the compass.

***************************************************************************************/

#include <Arduino_BMI270_BMM150.h>
#include <stdarg.h>

// TODO wiring clockwiseness
// TODO documenation of wiring and numbering of output pins and rotors, images, and USB protocol
// TODO method comments
// TODO Kalman Filter?
// TODO writing style check
// TODO get sensor data via usb protocol
// TODO fix translation from angles to rotor numbers

// Measurement Behaviour
int CALIBRATION_TICKS = 100; // how many measurements are conducted in the calibration phase
int CALIBRATION_TICK_DELAYS = 100; // the delay between the calibration ticks in milliseconds
int CALC_DIRECTION_TICK_DELAY = 100; // the delay between the measurement ticks in milliseconds
int MIN_MAG_MEASURE_DELAY_MILLIS = 0; // minimum delay between two magnetometer sensor readings
int MIN_ACC_MEASURE_DELAY_MILLIS = 0; // minimum delay between two accelerometer sensor readings
int mag_measure_sample_delay_millis, acc_measure_sample_delay_millis; // delay between two sensor readings caused by the hardware sample rate
int last_sensor_read_mag = 0;
int last_sensor_read_acc = 0;
int last_direction_calculation = 0;

// Filter data
int FILTER_BUFFER_SIZE = 10;
float filter_mag_buffer[10][3]; // stores past sensor data, first number is the buffer size of the filter
float filter_acc_buffer[10][3];

// Output Behaviour
int ANGULAR_CHANGE_THRESHOLD_AZI = 15; // the threshold change rate relative to last update that triggers a new update information output, in degrees
int ANGULAR_CHANGE_THRESHOLD_ALT = 15; // the threshold change rate relative to last update that triggers a new update information output, in degrees
int MAX_OUTPUT_DELAY_AZI = 5000; // the maximum delay between output updates (if nothing interesting happens) in milliseconds
int MAX_OUTPUT_DELAY_ALT = 5000; // the maximum delay between output updates (if nothing interesting happens) in milliseconds
int last_pin_output_time[12] = {0,0,0,0,0,0,0,0,0,0,0,0}; // at wich time the output for a pin was last requested
int last_pin_output_time_actual[12] = {0,0,0,0,0,0,0,0,0,0,0,0}; // at wich time the pin actually started outputting

// Test stuff
int TEST_ROTOR_ITERATIONS = 1;
int TEST_ROTOR_DELAY_MILLIS = 1000;

// Mechanical and wiring properties
bool IS_WIRING_REVERSED = false;
int IN_ANGLE_OFFSETS[3] = {0, 0, 0}; // the offsets to account for the position of the device relative to the body of the wearer (in deg)
int OUT_SMALLEST_OUT_PIN_NUMBER_AZI = 2; // the smallest used digital output pin-number for AZIMUTH
int OUT_SMALLEST_OUT_PIN_NUMBER_ALT = 0; // the smallest used digital output pin-number for ALTITUDE
int OUT_NUM_ROTORS_AZI = 8; // how many vibration rotor modules are used to represent 360 degrees AZIMUTH
int OUT_NUM_ROTORS_ALT = 0; // how many vibration rotor modules are used to represent 180 degrees ALTITUDE
int OUT_ROTOR_OFFSET_DEGREE_AZI = 180; // by how many degrees the rotors are offset, i.e. 0 if rotor 0 is in the front and 180 if it is in the back
int OUT_ROTOR_OFFSET_DEGREE_ALT = 90; // by how many degrees the rotors are offset, i.e. 0 if rotor 0 is in the middle and 90 if it is in tha top
int OUT_ROTOR_BURST_DURATION_MAX = 500; // how many miliseconds a vibration rotor stays active for outputing a single signal, if no other output is produced soon after
int OUT_ROTOR_BURST_DURATION_MIN = 500; // how many miliseconds a vibration rotor at least stays active for outputing a single signal even if another signal is produced soon after

float MATH_PI = 3.14159265; // good enough approximation

// sensor data variables
float raw_mag_vector[3]; // vec3 of raw magnetic field values
float norm_mag_vector[3]; // vec3 of normalized and cleaned magnetic direction values
float raw_acc_vector[3]; // vec3 of raw acceleration field values
float norm_acc_vector[3]; // vec3 of normalized and cleaned acceleration direction values
int hc_azi, hc_alt; // pointing direction in horizontal coordinates: azimuth [0, 360 deg] with 0 being north and 90 east etc., as well as altitude [-90, +90] with 0 being level and +90 Zenith

// calibration data magnetic
float calibration_mag_x[100]; // all x values measured during calibration
float calibration_mag_y[100]; // all y values measured during calibration
float calibration_mag_z[100]; // all z values measured during calibration
float cali_mag_max[3]; // the maximum value out of all calibration values, for each axis respectively
float cali_mag_min[3]; // the minimum value out of all calibration values, for each axis respectively
float cali_mag_avg[3]; // the arithmetic average value out of all calibration values, for each axis respectively
float cali_mag_std[3]; // the standard deviation of all calibration values, for each axis respectively
float offset_mag[3]; // the center of the calibration data points, i.e. the offset of the magnetic values

int last_output_angle_degrees_azi; // the output angle that was last provided for azimuth
int last_output_angle_degrees_alt; // the output angle that was last provided for altitude
int last_output_timestamp_azi; // when we last provided output for azimuth
int last_output_timestamp_alt; // when we last provided output for altitude

bool is_usb_mode; // debug mode for USB communication, can be switched on and off later
bool is_roto_output_switched_on; // if the rotors should actually be activated
bool is_spamming_degrees; // debug mode where current degree calculations are spammed
bool is_silent_mode; // disables all Serial output


/*******************************************************************************************/

/**
 * Function: calibrate_sensors
 * -----------------------
 * Calibrates the sensors by gathering a number of values to find the minimum, maximum, and average values.
 * Calculates various calibration results and prints them to the serial output.
 */
void calibrate_sensors() {
  printToSerial("s", "=======================================\nCalibrating\n");

  // gather a number of values to find min, max and avg
  for (int i=0; i<CALIBRATION_TICKS; i++) {
    printToSerial("sis", "Calibration tick ", i, ": ");
    
    if (IMU.magneticFieldAvailable()) {
      IMU.readMagneticField(raw_mag_vector[0], raw_mag_vector[1], raw_mag_vector[2]);
      print_dimensions(raw_mag_vector);
      calibration_mag_x[i] = raw_mag_vector[0];
      calibration_mag_y[i] = raw_mag_vector[1];
      calibration_mag_z[i] = raw_mag_vector[2];
    }
    delay(CALIBRATION_TICK_DELAYS);
  }

  cali_mag_max[0] = get_max(calibration_mag_x, CALIBRATION_TICKS);
  cali_mag_max[1] = get_max(calibration_mag_y, CALIBRATION_TICKS);
  cali_mag_max[2] = get_max(calibration_mag_z, CALIBRATION_TICKS);
  
  cali_mag_min[0] = get_min(calibration_mag_x, CALIBRATION_TICKS);
  cali_mag_min[1] = get_min(calibration_mag_y, CALIBRATION_TICKS);
  cali_mag_min[2] = get_min(calibration_mag_z, CALIBRATION_TICKS);
  
  cali_mag_avg[0] = get_avg(calibration_mag_x, CALIBRATION_TICKS);
  cali_mag_avg[1] = get_avg(calibration_mag_y, CALIBRATION_TICKS);
  cali_mag_avg[2] = get_avg(calibration_mag_z, CALIBRATION_TICKS);
  
  cali_mag_std[0] = get_std(calibration_mag_x, CALIBRATION_TICKS);
  cali_mag_std[1] = get_std(calibration_mag_y, CALIBRATION_TICKS);
  cali_mag_std[2] = get_std(calibration_mag_z, CALIBRATION_TICKS);
  
  offset_mag[0] = ((cali_mag_max[0] - cali_mag_min[0]) / 2) + cali_mag_min[0];
  offset_mag[1] = ((cali_mag_max[1] - cali_mag_min[1]) / 2) + cali_mag_min[1];
  offset_mag[2] = ((cali_mag_max[2] - cali_mag_min[2]) / 2) + cali_mag_min[2];
  
  printToSerial("s", "Calibration done!\n");
  print_calibration_results("mag_", calibration_mag_x, calibration_mag_y, calibration_mag_z, cali_mag_min, cali_mag_max, cali_mag_avg, cali_mag_std, offset_mag);
}

/**
 * Function: print_calibration_results
 * -----------------------
 * Prints the calibration results for a specific sensor (magnetometer) to the serial output.
 *
 * @param prefix: The prefix for the sensor data.
 * @param data_x: The array of x-axis sensor data.
 * @param data_y: The array of y-axis sensor data.
 * @param data_z: The array of z-axis sensor data.
 * @param data_min: The array of minimum values for each axis.
 * @param data_max: The array of maximum values for each axis.
 * @param data_avg: The array of average values for each axis.
 * @param data_std: The array of standard deviation values for each axis.
 * @param data_offset: The array of offset values for each axis.
 */
void print_calibration_results(char prefix[], float data_x[], float data_y[], float data_z[], float data_min[], float data_max[], float data_avg[], float data_std[], float data_offset[]){
  printToSerial("ssfsfsfsfsfs", prefix, "x=[", data_avg[0], " +- ", data_std[0], ", ", data_min[0], " - ", data_max[0], ", offset=", data_offset[0], "]\t");
  printToSerial("sfsfsfsfsfs", "y=[", data_avg[1], " +- ", data_std[1], ", ", data_min[1], " - ", data_max[1], ", offset=", data_offset[1], "]\t");
  printToSerial("sfsfsfsfsfs", "z=[", data_avg[2], " +- ", data_std[2], ", ", data_min[2], " - ", data_max[2], ", offset=", data_offset[2], "]\n");

  printToSerial("ss", prefix, "X_CALIBRATION_DATA = [");
  for(int i = 0; i < CALIBRATION_TICKS-1; i++){
    printToSerial("fs", data_x[i], ", ");
  }
  printToSerial("fs", data_x[CALIBRATION_TICKS-1], "]\n");

  printToSerial("ss", prefix, "Y_CALIBRATION_DATA = [");
  for(int i = 0; i < CALIBRATION_TICKS-1; i++){
    printToSerial("fs", data_y[i], ", ");
  }
  printToSerial("fs", data_y[CALIBRATION_TICKS-1], "]\n");

  printToSerial("ss", prefix, "Z_CALIBRATION_DATA = [");
  for(int i = 0; i < CALIBRATION_TICKS-1; i++){
    printToSerial("fs", data_z[i], ", ");
  }
  printToSerial("fs", data_z[CALIBRATION_TICKS-1], "]\n");
}

/**
 * Function: print_dimensions
 * -----------------------
 * Prints the dimensions (x, y, z) of a vector to the serial output.
 *
 * @param a: The vector array.
 */
void print_dimensions(float * a){
  printToSerial("sfsfsfs", "x=", a[0], "\ty=", a[1], "\tz=", a[2], "\n");
}

/**
 * Function: handle_accelerometer_input
 * -----------------------
 * Handles the input from the accelerometer sensor.
 * Reads raw acceleration values, normalizes them, and applies necessary transformations.
 * Stores the filtered value in the accelerometer buffer.
 */
void handle_accelerometer_input(){
  float x,y,z;

  // read raw acceleration values
  IMU.readAcceleration(x, y, z);
  raw_acc_vector[0] = x;
  raw_acc_vector[1] = y;
  raw_acc_vector[2] = z;

  // transform into a coordinate system that we can better work with:
  // The vector direction will point into the direction of "up" now,
  // with the coordinate system matching what we set the magnetometer values to.
  raw_acc_vector[1] = -raw_acc_vector[1];

  // Normalize, offset correction not necessary for accelerometer
  normalize_vec3(raw_acc_vector, norm_acc_vector);

  // Rotate to account for default device orientation
  rotate_system(norm_acc_vector, IN_ANGLE_OFFSETS[0], IN_ANGLE_OFFSETS[1], IN_ANGLE_OFFSETS[2], norm_acc_vector);

  filter_store_value(norm_acc_vector, filter_acc_buffer);
}

/**
 * Function: handle_compass_input
 * -----------------------
 * Handles the input from the compass (magnetometer) sensor.
 * Reads raw magnetic values, corrects offset, normalizes, and rotates the data to get reasonable results.
 * Stores the filtered value in the magnetometer buffer.
 */
void handle_compass_input(){
  float x,y,z;

  // read raw magnetic values
  IMU.readMagneticField(x, y, z);
  raw_mag_vector[0] = x;
  raw_mag_vector[1] = y;
  raw_mag_vector[2] = z;

  // correct offset, normalize and rotate raw data to get to reasonable results
  sub_vec3(raw_mag_vector, offset_mag, norm_mag_vector);
  normalize_vec3(norm_mag_vector, norm_mag_vector);
  
  // transform into a coordinate system that we can better work with:
  // The vector direction will point into the direction of "magnetic south" (~= geographic north) now,
  // with the coordinate system matching what we set the magnetometer values to.
  norm_mag_vector[2] = -norm_mag_vector[2];
  norm_mag_vector[1] = -norm_mag_vector[1];
  rotate_around_z(norm_mag_vector, MATH_PI / 2.0, norm_mag_vector);

  // Rotate to account for default device orientation
  rotate_system(norm_mag_vector, IN_ANGLE_OFFSETS[0], IN_ANGLE_OFFSETS[1], IN_ANGLE_OFFSETS[2], norm_mag_vector);
  
  filter_store_value(norm_mag_vector, filter_mag_buffer);
}

/**
 * Function: calculate_direction
 * -----------------------
 * Calculates the direction based on magnetometer and accelerometer data.
 * Calculates helper vectors and determines the azimuth and altitude.
 * Handles the resulting values for further processing.
 *
 * @param mag_direction: The magnetometer direction vector.
 * @param acc_direction: The accelerometer direction vector.
 */
void calculate_direction(float mag_direction[], float acc_direction[]){
  float x_vec[3] = {1, 0, 0}; // x axis is pointing straight forward from the device (by our definition)

  // calculate helper vectors that define certain directions relative to the device
  float untilted_west[3]; // geographic west direction horizontal to the ground
  float untilted_north[3]; // geographic north direction horizontal to the ground
  cross_product_vec3(acc_direction, mag_direction, untilted_west);
  normalize_vec3(untilted_west, untilted_west);
  cross_product_vec3(untilted_west, acc_direction, untilted_north);
  normalize_vec3(untilted_north, untilted_north);

  float horizontal_right[3]; // horizontal vector pointing leftwards from the device
  float horizontal_front[3]; // horizontal vector pointing forwards from the device
  cross_product_vec3(x_vec, acc_direction, horizontal_right);
  normalize_vec3(horizontal_right, horizontal_right);
  cross_product_vec3(acc_direction, horizontal_right, horizontal_front);
  normalize_vec3(horizontal_front, horizontal_front);

  // Azimuth = angle of the adjusted horizontal north direction to the adjusted horizontal front of the device
  hc_azi = radians_to_degrees(angle_between_vec3_dot(untilted_north, horizontal_front));
  
  // because we want the angle to be within [0, 360] not [0, 180], we check if it is on the western side
  if (radians_to_degrees(angle_between_vec3_dot(untilted_west, x_vec)) < 90){
    hc_azi = 360 - hc_azi;
  }

  // Altitude = 90 - angle between up direction and unadjusted forward (x-Axis)
  hc_alt = 90 - radians_to_degrees(angle_between_vec3_dot(acc_direction, x_vec));
  
  handle_degrees(hc_azi, hc_alt, "COMPASS");
}