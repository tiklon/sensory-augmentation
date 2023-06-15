/**************************************************************************************

File: auxiliary.ino
Author: Tiklon
Date: 05.06.2023
Description: This file contains various Arduino functions for simple mathematical
             operations and data manipulation.

***************************************************************************************/

/**
 * Function: get_sum
 * -----------------------
 * Calculates the sum of an array of floats.
 * 
 * @param a - The array of floats.
 * @param len - The length of the array.
 * @return The sum of the array elements.
 */
float get_sum(float a[], int len){
  double sum = 0.0;
  for (int i = 0 ; i < len; i++){
    sum += a[i];
  }
  return sum;
}

/**
 * Function: get_avg
 * -----------------------
 * Calculates the average value of an array of floats.
 * 
 * @param a - The array of floats.
 * @param len - The length of the array.
 * @return The average value of the array elements.
 */
float get_avg(float a[], int len){
  return (float)(get_sum(a, len) / len);
}

/**
 * Function: get_std
 * -----------------------
 * Calculates the standard deviation of an array of floats.
 * 
 * @param a - The array of floats.
 * @param len - The length of the array.
 * @return The standard deviation of the array elements.
 */
float get_std(float a[], int len) {
  float average = get_avg(a, len);
  long total = 0;
  for (int i = 0; i < len; i++) {
    total = total + (a[i] - average) * (a[i] - average);
  }

  float variance = total/(float)len;
  float std_dev = sqrt(variance);
  return std_dev;
}

/**
 * Function: get_min
 * -----------------------
 * Finds the minimum value in an array of floats.
 * 
 * @param a - The array of floats.
 * @param len - The length of the array.
 * @return The minimum value in the array.
 */
float get_min (float a[], int len){
  float minimum = a[0];
  for (int i = 0 ; i < len ; i++)
    minimum = min(a[i],minimum);
  return minimum;
}

/**
 * Function: get_max
 * -----------------------
 * Finds the maximum value in an array of floats.
 * 
 * @param a - The array of floats.
 * @param len - The length of the array.
 * @return The maximum value in the array.
 */
float get_max (float a[], int len){
  float maximum = a[0];
  for (int i = 0 ; i < len ; i++)
    maximum = max(a[i],maximum);
  return maximum;
}

/**
 * Function: get_value_from_command
 * -----------------------
 * Extracts a value from a string based on a separator and index.
 * 
 * @param data - The input string.
 * @param separator - The separator character.
 * @param index - The index of the value to extract.
 * @return The extracted value as a String.
 */
String get_value_from_command(String data, char separator, int index){ 
  int found = 0; 
  int strIndex[] = {0, -1}; 
  int maxIndex = data.length()-1; 
   
  for(int i=0; i<=maxIndex && found<=index; i++){ 
    if(data.charAt(i)==separator || i==maxIndex){ 
        found++; 
        strIndex[0] = strIndex[1]+1; 
        strIndex[1] = (i == maxIndex) ? i+1 : i; 
    } 
  } 
     
  return found>index ? data.substring(strIndex[0], strIndex[1]) : ""; 
} 

/**
 * Function: degrees_to_radians
 * -----------------------
 * Converts degrees to radians.
 * 
 * @param degrees - The value in degrees.
 * @return The value converted to radians.
 */
float degrees_to_radians(int degrees){
  float radians = ((float) degrees * 71.0) / 4068.0;
  return radians;

  // Alternative implementation:
  // float radians = ((degrees % 360) / 360.0) * (2.0 * MATH_PI);
  // return radians;
}

/**
 * Function: radians_to_degrees
 * -----------------------
 * Converts radians to degrees.
 * 
 * @param radians - The value in radians.
 * @return The value converted to degrees.
 */
int radians_to_degrees(float radians){
  int degrees = (radians * 4068) / 71;
  return degrees;

  // Alternative implementation:
  // int degrees = (int)((radians / (2 * MATH_PI)) * 360.0) % 360;
  // return radians;
}

/**
 * Function: shift_vec3_array_right
 * -----------------------
 * Shifts a 3D vector array to the right by one position.
 * The first element is set to zero.
 * 
 * @param vec_array - The 3D vector array.
 * @param array_size - The size of the array.
 */
void shift_vec3_array_right(float vec_array[][3], int array_size) {
  for (int i = array_size - 1; i > 0; i--) {
    vec_array[i][0] = vec_array[i-1][0];
    vec_array[i][1] = vec_array[i-1][1];
    vec_array[i][2] = vec_array[i-1][2];
  }
  vec_array[0][0] = 0.0;
  vec_array[0][1] = 0.0;
  vec_array[0][2] = 0.0;
}