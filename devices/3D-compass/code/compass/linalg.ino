/**************************************************************************************

File: linalg.ino
Author: Tiklon
Date: 05.06.2023
Description: This file contains various Arduino C++ functions for vector operations.
             The functions perform rotations, additions, subtractions, dot product,
             cross product, magnitude, normalization, inversion, and angle calculations
             for 3D vectors.

***************************************************************************************/

/**
 * Function: rotate_around_x
 * -----------------------
 * Rotates a 3D vector around the X-axis by a given angle.
 * 
 * @param a - The input vector.
 * @param theta - The angle of rotation in radians.
 * @param result - The resulting rotated vector.
 */
void rotate_around_x(float a[], float theta, float result[]){
  float tmp[3] = {0, 0, 0}; // to make sure there are no overwrite errors if result vector is equal to a
  tmp[0] = a[0];
  tmp[1] = a[1] * cos(theta) - a[2] * sin(theta);
  tmp[2] = a[1] * sin(theta) + a[2] * cos(theta);
  result[0] = tmp[0];
  result[1] = tmp[1];
  result[2] = tmp[2];
}

/**
 * Function: rotate_around_y
 * -----------------------
 * Rotates a 3D vector around the Y-axis by a given angle.
 * 
 * @param a - The input vector.
 * @param theta - The angle of rotation in radians.
 * @param result - The resulting rotated vector.
 */
void rotate_around_y(float a[], float theta, float result[]){
  float tmp[3] = {0, 0, 0}; // to make sure there are no overwrite errors if result vector is equal to a
  tmp[0] = a[0] * cos(theta) + a[2] * sin(theta);
  tmp[1] = a[1];
  tmp[2] = a[2] * cos(theta) - a[0] * sin(theta);
  result[0] = tmp[0];
  result[1] = tmp[1];
  result[2] = tmp[2];
}

/**
 * Function: rotate_around_z
 * -----------------------
 * Rotates a 3D vector around the Z-axis by a given angle.
 *
 * @param a - The input vector.
 * @param theta - The angle of rotation in radians.
 * @param result - The resulting rotated vector.
 */
void rotate_around_z(float a[], float theta, float result[]){
  float tmp[3] = {0, 0, 0}; // to make sure there are no overwrite errors if result vector is equal to a
  tmp[0] = a[0] * cos(theta) - a[1] * sin(theta);
  tmp[1] = a[0] * sin(theta) + a[1] * cos(theta);
  tmp[2] = a[2];
  result[0] = tmp[0];
  result[1] = tmp[1];
  result[2] = tmp[2];
}

/**
 * Function: rotate_system
 * -----------------------
 * Rotates a 3D vector system by angles (in degrees) along the axes x, y, and z.
 * Individual rotations are performed in this order!
 *
 * @param a - The input vector system.
 * @param degrees_x - The rotation angle in degrees around the x-axis.
 * @param degrees_y - The rotation angle in degrees around the y-axis.
 * @param degrees_z - The rotation angle in degrees around the z-axis.
 * @param result - The resulting rotated vector system.
 */
void rotate_system(float a[], int degrees_x, int degrees_y, int degrees_z, float result[]){
  // rotates the normalized 3d coordinates by angles (in degrees) along the axes x, y and z
  
  // Around X-axis:
  float tmp1[3] = {0, 0, 0};
  rotate_around_x(a, degrees_to_radians(degrees_x), tmp1);
  
  // Around Y-axis:
  float tmp2[3] = {0, 0, 0};
  rotate_around_y(tmp1, degrees_to_radians(degrees_y), tmp2);
  
  // Around Z-axis:
  float tmp[3] = {0, 0, 0};
  rotate_around_z(tmp2, degrees_to_radians(degrees_z), result);
}

/**
 * Function: add_vec3
 * -----------------------
 * Adds two 3D vectors.
 *
 * @param a - The first input vector.
 * @param b - The second input vector.
 * @param result - The resulting vector after addition.
 */
void add_vec3(float a[], float b[], float result[]){
  result[0] = a[0] + b[0];
  result[1] = a[1] + b[1];
  result[2] = a[2] + b[2];
}

/**
 * Function: sub_vec3
 * -----------------------
 * Subtracts two 3D vectors.
 *
 * @param a - The first input vector.
 * @param b - The second input vector.
 * @param result - The resulting vector after subtraction.
 */
void sub_vec3(float a[], float b[], float result[]){
  result[0] = a[0] - b[0];
  result[1] = a[1] - b[1];
  result[2] = a[2] - b[2];
}

/**
 * Function: dot_product_vec3
 * -----------------------
 * Calculates the dot product of two 3D vectors.
 * 
 * @param a - The first input vector.
 * @param b - The second input vector.
 * @return The dot product of the two input vectors.
 */
float dot_product_vec3(float a[], float b[]){
    float result = 0;
 
    for (int i = 0; i < 3; i++){
      result = result + a[i] * b[i];
    }
    return result;
}

/**
 * Function: cross_product_vec3
 * -----------------------
 * Calculates the cross product of two 3D vectors.
 * 
 * @param a - The first input vector.
 * @param b - The second input vector.
 * @param result - The resulting vector after cross product calculation.
 */
void cross_product_vec3(float a[], float b[], float result[]){
  float tmp[3] = {0, 0, 0}; // to make sure there are no overwrite errors if result vector is equal to a or b
  tmp[0] = a[1] * b[2] - a[2] * b[1];
  tmp[1] = a[2] * b[0] - a[0] * b[2];
  tmp[2] = a[0] * b[1] - a[1] * b[0];
  result[0] = tmp[0];
  result[1] = tmp[1];
  result[2] = tmp[2];
}

/**
 * Function: magnitude_vec3
 * -----------------------
 * Calculates the magnitude of a 3D vector.
 * 
 * @param a - The input vector.
 * @return The magnitude of the input vector.
 */
float magnitude_vec3(float a[]){
  return sqrt((a[0] * a[0]) + (a[1] * a[1]) + (a[2] * a[2]));
}

/**
 * Function: normalize_vec3
 * -----------------------
 * Normalizes a 3D vector.
 * 
 * @param a - The input vector.
 * @param result - The resulting normalized vector.
 */
void normalize_vec3(float a[], float result[]){
  float mag = magnitude_vec3(a);
  result[0] = a[0]/mag;
  result[1] = a[1]/mag;
  result[2] = a[2]/mag;
}

/**
 * Function: invert_vec3
 * -----------------------
 * Inverts a 3D vector.
 * 
 * @param a - The input vector.
 * @param result - The resulting inverted vector.
 */
void invert_vec3(float a[], float result[]){
  result[0] = -a[0];
  result[1] = -a[1];
  result[2] = -a[2];
}

/**
 * Function: angle_between_vec3_dot
 * -----------------------
 * Calculates the angle (in radians) between two 3D vectors using the dot product.
 * 
 * @param a - The first input vector.
 * @param b - The second input vector.
 * @return The angle (in radians) between the two input vectors.
 */
float angle_between_vec3_dot(float a[], float b[]){
  return acos(dot_product_vec3(a, b) / (magnitude_vec3(a) * magnitude_vec3(b)));
}

/**
 * Function: angle_between_vec3_cross
 * -----------------------
 * Calculates the angle (in radians) between two 3D vectors using the cross product.
 * 
 * @param a - The first input vector.
 * @param b - The second input vector.
 * @return The angle (in radians) between the two input vectors.
 */
float angle_between_vec3_cross(float a[], float b[]){
  float cross[3] = {0, 0, 0};
  cross_product_vec3(a, b, cross);

  return asin(magnitude_vec3(cross) / (magnitude_vec3(a) * magnitude_vec3(b)));
}