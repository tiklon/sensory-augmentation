/**************************************************************************************

File: bracelet.ino
Author: Tiklon
Date: 05.06.2023
Description: This file contains all functions related to the output performed through
             the vibration motors of the bracelet

***************************************************************************************/

/**
 * Function: handle_degrees
 * -----------------------
 * Handles the azimuth and altitude degrees by calling the respective
 * handle_degrees_azi and handle_degrees_alt functions.
 *
 * @param degrees_azi: The azimuth degrees.
 * @param degrees_alt: The altitude degrees.
 * @param source: The source of the degrees.
 */
void handle_degrees(int degrees_azi, int degrees_alt, String source){
  handle_degrees_azi(degrees_azi, source);
  handle_degrees_alt(degrees_alt, source);
}

/**
 * Function: handle_degrees_azi
 * ---------------------------
 * Handles the azimuth degrees by comparing them with the previous degrees
 * and determining if the change or delay thresholds are met. If so, it
 * performs the necessary actions such as printing debug information,
 * vibrating the rotor, and updating the timestamp and degrees.
 *
 * @param degrees: The azimuth degrees.
 * @param source: The source of the degrees.
 */
void handle_degrees_azi(int degrees, String source){
  int prev = last_output_angle_degrees_azi;
  int degree_diff = min(max(prev, degrees) - min(prev, degrees), 360 + min(prev, degrees) - max(prev, degrees));

  if (is_spamming_degrees || ((degree_diff >= ANGULAR_CHANGE_THRESHOLD_AZI) || ((millis()-last_output_timestamp_azi) >= MAX_OUTPUT_DELAY_AZI))){
    
    // Debug serial info
    printToSerial("sssis", "[", source, "] AZI degrees=", degrees, "\n");

    // Vibrate
    if (is_roto_output_switched_on){
      output_angle_azi(degrees);
    }

    last_output_timestamp_azi = millis();
    last_output_angle_degrees_azi = degrees;
  }  
}

/**
 * Function: handle_degrees_alt
 * ---------------------------
 * Handles the altitude degrees by comparing them with the previous degrees
 * and determining if the change or delay thresholds are met. If so, it
 * performs the necessary actions such as printing debug information,
 * vibrating the rotor, and updating the timestamp and degrees.
 *
 * @param degrees: The altitude degrees.
 * @param source: The source of the degrees.
 */
void handle_degrees_alt(int degrees, String source){

  int prev = last_output_angle_degrees_alt;
  int degree_diff = min(max(prev, degrees) - min(prev, degrees), 180 + min(prev, degrees) - max(prev, degrees));
  
  if (is_spamming_degrees || ((degree_diff >= ANGULAR_CHANGE_THRESHOLD_ALT) || ((millis()-last_output_timestamp_alt) >= MAX_OUTPUT_DELAY_ALT))){
    
    // Debug serial info
    printToSerial("sssis", "[", source, "] ALT degrees=", degrees, "\n");

    // Vibrate
    if (is_roto_output_switched_on){
      output_angle_alt(degrees);
    }

    last_output_timestamp_alt = millis();
    last_output_angle_degrees_alt = degrees;
  }  
}

/**
 * Function: vibrate_relevant_rotor
 * --------------------------------
 * Determines the most recently requested and actually turned on rotor,
 * and turns off any pins that have exceeded their maximum on time.
 * It then determines which pin to turn on based on the requested and
 * actual rotor states and their respective time stamps, and performs
 * the necessary actions to turn on/off the relevant pins.
 */
void vibrate_relevant_rotor(){

  // find the motor that was most recently requested to be turned on and the relevant time
  int highest_index_requested = -1;
  int highest_time_requested = -1;
  for (int i = 0; i < 12; i++){
    if (last_pin_output_time[i] > highest_time_requested){
      highest_index_requested = i;
      highest_time_requested = last_pin_output_time[i];
    }
  }

  // find the motor that was most recently actually turned on and the relevant time
  int highest_index_actual = -1;
  int highest_time_actual = -1;
  for (int i = 0; i < 12; i++){
    if (last_pin_output_time_actual[i] > highest_time_actual){
      highest_index_actual = i;
      highest_time_actual = last_pin_output_time_actual[i];
    }
  }

  // Turn off any pins that have exceeded their maximum on time
  for (int i = 0; i < 12; i++) {
    if (millis() - last_pin_output_time_actual[i] > OUT_ROTOR_BURST_DURATION_MAX) {
      digitalWrite(i, LOW);
    }
  }

  // Determine which pin to turn on
  if (highest_index_requested != -1) {
    if (highest_index_actual != -1 && highest_time_requested > (highest_time_actual + OUT_ROTOR_BURST_DURATION_MIN)) {
      printToSerial("sisisis", "HIGH! ", highest_time_actual, ", ", highest_time_requested, " ", highest_time_actual, "\n");

      // Turn off the currently active pin
      digitalWrite(highest_index_actual, LOW);
      last_pin_output_time_actual[highest_index_actual] = 0;

      // Turn on the requested pin
      if (highest_index_requested = 4){
        digitalWrite(highest_index_requested, HIGH);
        last_pin_output_time[highest_index_requested] = 0;
        last_pin_output_time_actual[highest_index_requested] = millis();
      }
    }
  }
}

/**
 * Function: test_vibration_rotors
 * -------------------------------
 * Tests the vibration of rotors for both azimuth and altitude angles.
 * It iterates over the desired number of test iterations and calls
 * the vibrate_rotor_azi and vibrate_rotor_alt functions to vibrate
 * the respective rotors. It also includes delays and calls the
 * vibrate_relevant_rotor function to handle the relevant rotor vibrations.
 */
void test_vibration_rotors() {
  printToSerial("s", "=======================================\nTesting Rotors (AZIMUTH)\n");
  for (int i=0; i<TEST_ROTOR_ITERATIONS; i++) {
    for (int j=0; j<OUT_NUM_ROTORS_AZI; j++) {
      vibrate_rotor_azi(j);
      vibrate_relevant_rotor();
      delay(OUT_ROTOR_BURST_DURATION_MAX + 50);
      vibrate_relevant_rotor();
      delay(TEST_ROTOR_DELAY_MILLIS);
    }
  }
  printToSerial("s", "Testing Rotors (ALTITUDE)\n");
  for (int i=0; i<TEST_ROTOR_ITERATIONS; i++) {
    for (int j=0; j<OUT_NUM_ROTORS_ALT; j++) {
      vibrate_rotor_alt(j);
      vibrate_relevant_rotor();
      delay(OUT_ROTOR_BURST_DURATION_MAX + 50);
      vibrate_relevant_rotor();
      delay(TEST_ROTOR_DELAY_MILLIS);
    }
  }
}

/**
 * Function: output_angle_azi
 * --------------------------
 * Outputs the azimuth angle by vibrating the relevant rotor.
 * It calculates which rotor to turn based on the input degrees,
 * the number of azimuth rotors, and the degree range per rotor.
 *
 * @param degrees: The azimuth angle in degrees.
 */
void output_angle_azi(int degrees) {
  int rotor_index = degrees / (360 / OUT_NUM_ROTORS_AZI);
  vibrate_rotor_azi(rotor_index);
}

/**
 * Function: output_angle_alt
 * --------------------------
 * Outputs the altitude angle by vibrating the relevant rotor.
 * It calculates which rotor to turn based on the input degrees,
 * the number of altitude rotors, and the degree range per rotor.
 *
 * @param degrees: The altitude angle in degrees.
 */
void output_angle_alt(int degrees) {
  int rotor_index = degrees / (180 / OUT_NUM_ROTORS_ALT);
  vibrate_rotor_alt(rotor_index);
}

/**
 * Function: vibrate_rotor_azi
 * ---------------------------
 * Vibrates the specified azimuth rotor.
 * It determines the output pin number based on the rotor number and wiring configuration.
 *
 * @param rotor_number: The rotor number to vibrate.
 */
void vibrate_rotor_azi(int rotor_number) {
  int out_pin_nr = -1;
  if (IS_WIRING_REVERSED) {
    out_pin_nr = OUT_SMALLEST_OUT_PIN_NUMBER_AZI + rotor_number;
  } else {
    out_pin_nr = OUT_SMALLEST_OUT_PIN_NUMBER_AZI + OUT_NUM_ROTORS_AZI - 1 - rotor_number;
  }

  printToSerial("sisis", " - vibrating rotor ", rotor_number, " [pin= ", out_pin_nr, "]\n");

  last_pin_output_time[out_pin_nr] = millis();
}

/**
 * Function: vibrate_rotor_alt
 * ---------------------------
 * Vibrates the specified altitude rotor.
 * It determines the output pin number based on the rotor number and wiring configuration.
 *
 * @param rotor_number: The rotor number to vibrate.
 */
void vibrate_rotor_alt(int rotor_number) {
  int out_pin_nr = -1;
  if (IS_WIRING_REVERSED) {
    out_pin_nr = OUT_SMALLEST_OUT_PIN_NUMBER_ALT + rotor_number;
  } else {
    out_pin_nr = OUT_SMALLEST_OUT_PIN_NUMBER_ALT + OUT_NUM_ROTORS_ALT - 1 - rotor_number;
  }

  printToSerial("sisis", " - vibrating rotor ", rotor_number, " [pin= ", out_pin_nr, "]\n");

  last_pin_output_time[out_pin_nr] = millis();
}
