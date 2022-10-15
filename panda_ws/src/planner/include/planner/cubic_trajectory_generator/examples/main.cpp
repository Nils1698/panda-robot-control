//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: main.cpp
//
// MATLAB Coder version            : 5.3
// C/C++ source code generated on  : 12-May-2022 11:56:34
//

/*************************************************************************/
/* This automatically generated example C++ main file shows how to call  */
/* entry-point functions that MATLAB Coder generated. You must customize */
/* this file for your application. Do not modify this file directly.     */
/* Instead, make a copy of this file, modify it, and integrate it into   */
/* your development environment.                                         */
/*                                                                       */
/* This file initializes entry-point function arguments to a default     */
/* size and value before calling the entry-point functions. It does      */
/* not store or use any values returned from the entry-point functions.  */
/* If necessary, it does pre-allocate memory for returned values.        */
/* You can use this file as a starting point for a main function that    */
/* you can deploy in your application.                                   */
/*                                                                       */
/* After you copy the file, and before you deploy it, you must make the  */
/* following changes:                                                    */
/* * For variable-size function arguments, change the example sizes to   */
/* the sizes that your application requires.                             */
/* * Change the example values of function arguments to the values that  */
/* your application requires.                                            */
/* * If the entry-point functions return values, store these values or   */
/* otherwise use them as required by your application.                   */
/*                                                                       */
/*************************************************************************/

// Include Files
#include "main.h"
#include "cubic_trajectory_generator.h"
#include "cubic_trajectory_generator_terminate.h"
#include "rt_nonfinite.h"

// Function Declarations
static void argInit_2x3_real_T(double result[6]);

static double argInit_real_T();

static void main_cubic_trajectory_generator();

// Function Definitions
//
// Arguments    : double result[6]
// Return Type  : void
//
static void argInit_2x3_real_T(double result[6])
{
  // Loop over the array to initialize each element.
  for (int idx0{0}; idx0 < 2; idx0++) {
    for (int idx1{0}; idx1 < 3; idx1++) {
      // Set the value of the array element.
      // Change this value to the value that the application requires.
      result[idx0 + (idx1 << 1)] = argInit_real_T();
    }
  }
}

//
// Arguments    : void
// Return Type  : double
//
static double argInit_real_T()
{
  return 0.0;
}

//
// Arguments    : void
// Return Type  : void
//
static void main_cubic_trajectory_generator()
{
  double trajectory[60];
  double wayPoints_tmp[6];
  // Initialize function 'cubic_trajectory_generator' input arguments.
  // Initialize function input argument 'wayPoints'.
  argInit_2x3_real_T(wayPoints_tmp);
  // Initialize function input argument 'wayPointVels'.
  // Call the entry-point 'cubic_trajectory_generator'.
  cubic_trajectory_generator(wayPoints_tmp, wayPoints_tmp, trajectory);
}

//
// Arguments    : int argc
//                char **argv
// Return Type  : int
//
int main(int, char **)
{
  // The initialize function is being called automatically from your entry-point
  // function. So, a call to initialize is not included here. Invoke the
  // entry-point functions.
  // You can call entry-point functions multiple times.
  main_cubic_trajectory_generator();
  // Terminate the application.
  // You do not need to do this more than one time.
  cubic_trajectory_generator_terminate();
  return 0;
}

//
// File trailer for main.cpp
//
// [EOF]
//
