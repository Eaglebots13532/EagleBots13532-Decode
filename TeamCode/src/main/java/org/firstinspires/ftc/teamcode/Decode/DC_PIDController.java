// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Decode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class DC_PIDController {

  double spdMax = .8;
  double Kp;
  double Ki;
  double Kd;
  double error = 0.0;
  double lastError = 0.0;
  double integral = 0.0;
  double derivative = 0.0;
  boolean angleWrap = false;

  ElapsedTime timeS = new ElapsedTime();

  /**
   * Set PID gains
   *
   * @param Kp proportional gain
   * @param Ki integral gain
   * @param Kd derivative gain
   */
  public DC_PIDController(double Kp, double Ki, double Kd) {
    this.Kp = Kp;
    this.Ki = Ki;
    this.Kd = Kd;
  } // constructor PIDController

  public DC_PIDController(double Kp, double Ki, double Kd, boolean angleWrap) {
    this.Kp = Kp;
    this.Ki = Ki;
    this.Kd = Kd;
    this.angleWrap = angleWrap;
  } // constructor PIDController with angle

  /**
   * calculate PID output given the reference and the current system state
   *
   * @param reference where we would like our system to be
   * @param state where our system is
   * @return the signal to send to our motor or other actuator
   */
  public double output(double reference, double state) {
    // reference - goal(encoder counts to drive to
    // state - the present encoder count
    double delta = reference - state;
    double converge = delta / reference;
    // check if we need to unwrap angle
    if (angleWrap) {
      error = angleWrap(delta);
    } else {
      error = delta;
    }
    // forward euler integration
    integral += error * timeS.seconds();
    derivative = (error - lastError) / timeS.seconds();

    double output = (error * Kp) + (integral * Ki) + (derivative * Kd);
    if (output > spdMax) output = spdMax; // force output to with in max motor speed ~ <= 1
    if (output < 0.2 || error < 1.0) output = 0.0;
    lastError = error;
    timeS.reset();
    // percent of completion

    return output;
  } // output

  public double angleWrap(double radians) {
    while (radians > Math.PI) {
      radians -= 2 * Math.PI;
    }
    while (radians < -Math.PI) {
      radians += 2 * Math.PI;
    }
    return radians;
  } // angleWrap
} // end class PIDController
