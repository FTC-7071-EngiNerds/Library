#pragma systemFile	// eliminates warnings about unused functions

//
// pid.c - PID controller related functions
//
// Copyright (c) 2013 Domenic Rodriguez. All rights reserved.
//

//
// TPID
//
// Contains all of the data related to an individual PID controller.
//
// Fields:
// int kP - Proportional constant
// int kI - Integral constant
// int kD - Derivative constant
// int integral - the integral component, or the sum of the error over time
// int prevError - the previous error, used for computing the derivative
//
typedef struct
{
	int kP;
	int kI;
	int kD;
	int integral;
	int prevError;
} TPID;

//
// Setup the PID data structure for use by the other functions
//
// Parameters:
// TPID &pid - the PID controller data (passed by reference)
// int kP - Proportional constant
// int kI - Integral constant
// int kD - Derivative constant
//
void PID_Init(TPID &pid, int kP, int kI, int kD)
{
	// Set the P, I, and D constants
	pid.kP = kP;
	pid.kI = kI;
	pid.kD = kD;

	// Initialize the other fields
	pid.integral = 0;
	pid.prevError = 0;
}

//
// Run one iteration of the PID controller
//
// Parameters:
// TPID &pid - the PID controller data (passed by reference)
// int setpoint - the target value of the system
// int processVariable - the current value of the system
// int deltaTime - the ellapsed time since the last iteration of the function
//
// Returns:
// The output of the PID equation
//
int PID_Run(TPID &pid, int setpoint, int processVariable, int deltaTime)
{
	// Proportional
	int error = setpoint - processVariable;

	// Integral
	// The sum of the error over time
	pid.integral += (error * deltaTime);

	// Derivative
	// The slope of the error
	int derivative = (error - pid.prevError) / deltaTime;

	// Result is the sum of the three terms multiplied by their respective constants
	int output = (error * pid.kP) + (pid.integral * pid.kI) + (derivative * pid.kD);

	// Return the result
	return output;}
