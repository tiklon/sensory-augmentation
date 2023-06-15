/**************************************************************************************

File: main.ino
Author: Tiklon
Date: 05.06.2023
Description: This file contains the main project workflow and core functionality

***************************************************************************************/

/**
 * Function: setup
 * ----------------
 * Initializes the program by setting up the serial communication, sensors, vibration modules,
 * and other variables used in the program.
 * It also calibrates the sensors and sets initial values for the variables.
 * This function is called once at the start of the program.
 */
void setup() {
  // Initialize cmd output
  Serial.begin(9600);
  printToSerial("s", "\n\nStarted\n");

  // Initialize sensors
  IMU.begin();

  // Print sensor information
  mag_measure_sample_delay_millis = 1000 / IMU.magneticFieldSampleRate();
  printToSerial("sis", "Magnetic field sample delay = ", mag_measure_sample_delay_millis, "\n");
  acc_measure_sample_delay_millis = 1000 / IMU.accelerationSampleRate();
  printToSerial("ss", "Acceleration sample delay = ", acc_measure_sample_delay_millis, "\"");

  // Initialize vibration modules
  for (int i = 0; i < OUT_NUM_ROTORS_AZI; i++) {
    pinMode(OUT_SMALLEST_OUT_PIN_NUMBER_AZI + i, OUTPUT);
  }
  for (int i = 0; i < OUT_NUM_ROTORS_ALT; i++) {
    pinMode(OUT_SMALLEST_OUT_PIN_NUMBER_ALT + i, OUTPUT);
  }
  test_vibration_rotors();

  // Calibrate sensors
  calibrate_sensors();

  last_output_angle_degrees_azi = 0;
  last_output_angle_degrees_alt = 0;
  last_output_timestamp_azi = 0;
  last_output_timestamp_alt = 0;

  // USB-mode will indicate that the device gets the degree data via USB interface instead of the onboard sensors
  is_usb_mode = false;

  // Turning the rotors off and degree spamming on can be useful for debugging
  is_roto_output_switched_on = true;
  is_spamming_degrees = true;
  is_silent_mode = false;
}

/**
 * Function: printToSerial
 * -----------------------
 * Prints formatted text to the serial monitor.
 * It takes a format string and variable arguments.
 * The format string specifies the types of arguments passed.
 * The function supports the 's' format specifier for strings, 'f' for floats,
 * and 'i' for integers.
 */
void printToSerial(const char* format, ...) {
  if (is_silent_mode) {
    return;
  }

  va_list args; // Initialize argument list
  va_start(args, format); // Start variable arguments

  while (*format) {
    if (*format == 's') {
      char *str = va_arg(args, char*);
      Serial.print(str);
    } else if (*format == 'f') { // Handle float argument
      float f = va_arg(args, double);
      Serial.print(f);
    } else if (*format == 'i') { // Handle integer argument
      int i = va_arg(args, int);
      Serial.print(i);
    }
    format++;
  }

  va_end(args); // End variable arguments
}

/**
 * Function: handle_usb_input
 * --------------------------
 * Handles the input received from the USB interface.
 * It reads the incoming data from the serial buffer, processes the commands,
 * and performs the corresponding actions.
 * The function checks for different commands related to USB control, degrees,
 * rotor output, and spamming degrees, and executes the appropriate actions.
 */
void handle_usb_input() {
  // Read the oldest byte in the serial buffer
  String incomingString = Serial.readString();
  incomingString.trim();
  printToSerial("sss", ">>>> reading: ", incomingString, "\n");

  // Switch action based on received byte
  if (incomingString == "USB-CONTROL=ON") {
    is_usb_mode = true;
    printToSerial("s", "[USB] ON\n");
  } else if (incomingString == "USB-CONTROL=OFF") {
    is_usb_mode = false;
    printToSerial("s", "[USB] OFF\n");
  } else if (incomingString.startsWith("USB-DEGREES-AZI=")) {
    // Make sure USB-control is actually turned on
    if (is_usb_mode) {
      int degrees = get_value_from_command(incomingString, '=', 1).toInt();
      handle_degrees_azi(degrees, "USB");
    } else {
      printToSerial("s", "[USB] ERROR: Detected degree command, but USB-Control is turned off\n");
    }
  } else if (incomingString.startsWith("USB-DEGREES-ALT=")) {
    // Make sure USB-control is actually turned on
    if (is_usb_mode) {
      int degrees = get_value_from_command(incomingString, '=', 1).toInt();
      handle_degrees_alt(degrees, "USB");
    } else {
      printToSerial("s", "[USB] ERROR: Detected degree command, but USB-Control is turned off\n");
    }
  } else if (incomingString == "ROTORS=ON") {
    is_roto_output_switched_on = true;
    printToSerial("s", "[USB] rotor output turned on\n");
  } else if (incomingString == "ROTORS=OFF") {
    is_roto_output_switched_on = false;
    printToSerial("s", "[USB] rotor output turned off\n");
  } else if (incomingString == "SPAM-DEGREES=ON") {
    is_spamming_degrees = true;
    printToSerial("s", "[USB] spamming degrees turned on\n");
  } else if (incomingString == "SPAM-DEGREES=OFF") {
    is_spamming_degrees = false;
    printToSerial("s", "[USB] spamming degrees turned off\n");
  } else {
    printToSerial("sss", "[USB] unknown command: ", incomingString, "\n");
  }
}

/**
 * Function: loop
 * --------------
 * The main loop of the program that runs continuously.
 * It checks for incoming serial data, reads sensor data, performs calculations,
 * and controls the output of the program.
 * The loop also handles timing and prints debug information.
 */
void loop() {
  int time_before = millis();

  // Check for incoming serial data
  if (Serial.available() > 0) {
    handle_usb_input();
  }

  // Get sensor data (if available)
  if (!is_usb_mode) {
    if (IMU.accelerationAvailable() &&
        ((millis() - last_sensor_read_acc) > MIN_ACC_MEASURE_DELAY_MILLIS) &&
        ((millis() - last_sensor_read_acc) > acc_measure_sample_delay_millis)) {
      handle_accelerometer_input();
      last_sensor_read_acc = millis();
    }
    if (IMU.magneticFieldAvailable() &&
        ((millis() - last_sensor_read_mag) > MIN_MAG_MEASURE_DELAY_MILLIS) &&
        ((millis() - last_sensor_read_mag) > mag_measure_sample_delay_millis)) {
      handle_compass_input();
      last_sensor_read_mag = millis();
    }
  }
  // TODO buffer here

  // Get direction and make output
  if ((millis() - last_direction_calculation) > CALC_DIRECTION_TICK_DELAY) {
    float filtered_mag[3] = {0.0, 0.0, 0.0};
    float filtered_acc[3] = {0.0, 0.0, 0.0};

    if (!is_usb_mode) {
      filter_exp_weigthed_moving_average(filter_mag_buffer, FILTER_BUFFER_SIZE, filtered_mag);
      filter_exp_weigthed_moving_average(filter_acc_buffer, FILTER_BUFFER_SIZE, filtered_acc);
    } else {
      unfiltered_from_buffer(filter_mag_buffer, filtered_mag);
      unfiltered_from_buffer(filter_acc_buffer, filtered_acc);
    }

    calculate_direction(norm_mag_vector, norm_acc_vector);
    last_direction_calculation = millis();
  }

  // Power the pin relevant to the recently turned on rotor
  vibrate_relevant_rotor();

  int time_after = millis();
  // printToSerial("sis", "Timing of loop: ", time_after - time_before, "ms");
}