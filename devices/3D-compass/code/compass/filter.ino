/**************************************************************************************

File: filter.ino
Author: Tiklon
Date: 05.06.2023
Description: This file contains implementation of filtering functions for signal processing.

***************************************************************************************/

/**
 * Function: filter_store_value
 * -----------------------
 * Stores a value in a buffer by shifting the existing values to the right.
 * The value is stored at the beginning of the buffer.
 * 
 * @param value: The value to be stored.
 * @param buffer: The buffer to store the value in.
 */
void filter_store_value(float value[3], float buffer[][3]){
  shift_vec3_array_right(buffer, FILTER_BUFFER_SIZE);
  buffer[0][0] = value[0];
  buffer[0][1] = value[1];
  buffer[0][2] = value[2];
}

/**
 * Function: unfiltered_from_buffer
 * -----------------------
 * Retrieves the unfiltered value from a buffer.
 * The value at the beginning of the buffer is retrieved.
 * 
 * @param buffer: The buffer containing the values.
 * @param result: The resulting unfiltered value.
 */
void unfiltered_from_buffer(float buffer[][3], float result[3]){
  result[0] = buffer[0][0];
  result[1] = buffer[0][1];
  result[2] = buffer[0][2];
}

/**
 * Function: filter_moving_average
 * -----------------------
 * Applies a moving average filter to a buffer of values.
 * The filter calculates the average of a window of values from the buffer.
 * 
 * @param buffer: The buffer containing the values.
 * @param window_size: The size of the moving average window.
 * @param result: The resulting filtered value.
 */
void filter_moving_average(float buffer[][3], int window_size, float result[3]){
  // window_size must be smaller than or equal to the buffer size
  float sum[3] = {0.0, 0.0, 0.0};
  float average[3] = {0.0, 0.0, 0.0};
  
  for (int i = 0; i < window_size; i++) {
    sum[0] += buffer[i][0];
    sum[1] += buffer[i][1];
    sum[2] += buffer[i][2];
  }
  
  average[0] = sum[0] / window_size;
  average[1] = sum[1] / window_size;
  average[2] = sum[2] / window_size;

  // normalize result vector length
  normalize_vec3(average, average);
  
  result[0] = average[0];
  result[1] = average[1];
  result[2] = average[2];
}

/**
 * Function: filter_exp_weigthed_moving_average
 * -----------------------
 * Applies an exponentially weighted moving average filter to a buffer of values.
 * The filter calculates the weighted average of a window of values from the buffer,
 * with the weights decreasing exponentially.
 * 
 * @param buffer: The buffer containing the values.
 * @param window_size: The size of the exponentially weighted moving average window.
 * @param result: The resulting filtered value.
 */
void filter_exp_weigthed_moving_average(float buffer[][3], int window_size, float result[3]){
  // window_size must be smaller than or equal to the buffer size
  float sum[3] = {0.0, 0.0, 0.0};
  
  for (int i = 0; i < window_size; i++) {
    sum[0] += buffer[i][0] * (1.0/(float) pow(2, i));
    sum[1] += buffer[i][1] * (1.0/(float) pow(2, i));
    sum[2] += buffer[i][2] * (1.0/(float) pow(2, i));
  }

  // normalize result vector length
  normalize_vec3(sum, sum);

  result[0] = sum[0];
  result[1] = sum[1];
  result[2] = sum[2];
}