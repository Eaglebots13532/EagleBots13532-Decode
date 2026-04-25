// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Decode;

import org.firstinspires.ftc.teamcode.drivers.GameDriver;
import org.firstinspires.ftc.teamcode.drivers.odo.CarlOdometryExampleImplementation;

public class CarlHoodShoot {

  GameDriver motors;

  CarlOdometryExampleImplementation odo;

  public CarlHoodShoot(GameDriver motors, CarlOdometryExampleImplementation odo) {
    this.motors = motors;
    this.odo = odo;
  }

  double angleError;
  double lastAngleError;
  double hoodAngle;
  double hoodIntegral;
  double hoodKP = 0.05;
  double hoodKI = 0.00;
  double hoodKD = 0.005;
  double lastTime;
  double changeTime;
  // Parameters for speed of ball equation
  final double massBall = 75; // in grams
  final double radiusBallPath =
      (104 / 2)
          / 10; // Radius of the ball during the path of acceleration in the shooter I believe, in
  // Cm
  final double flyWheelMOI = 388.8; // Moment of inertia of the flywheel, g * cm^2
  final double flyWheelMass = 675; // Flywheel mass in grams
  boolean isHoming = false;

  public double getHoodServoPowerPID(double seekAngle, double elapsedTime) {
    double derivative;
    hoodAngle =
        motors.getHoodEncoderPos()
            * 35.95
            / 1117
            * 4
            * 90
            / 100 /*Difference between carl's code and eagelbots code compensation*/
            * 360
            / 8192;
    angleError = seekAngle - hoodAngle;

    changeTime = elapsedTime - lastTime;

    derivative = (angleError - lastAngleError) / changeTime;

    hoodIntegral += angleError * changeTime;
    hoodIntegral =
        Math.max(
            -0.1, Math.min(0.1, hoodIntegral)); // clamp integral to keep from exploading in value

    double out = ((hoodKP * angleError) + (hoodKI * hoodIntegral) + (hoodKD * derivative));

    lastAngleError = angleError;
    lastTime = elapsedTime;

    return out;
  }

  public double getHoodAngle() {
    return hoodAngle;
  }

  public double getAngleError() {
    return angleError;
  }

  double theta1;
  double theta2;

  public double getLaunchAngle(double x, double y, double velocity, boolean isTheta1) {
    double g = 9.81;

    // Sets up quadratic
    double a = (g * x * x) / (2 * velocity * velocity);
    double b = -x;
    double c = y + a;

    // Evaluates the square root part of the quadratic equation
    double discriminant = b * b - 4 * a * c;

    if (discriminant < 0) {
      return Double.NaN; // No valid shot
    }

    double sqrt = Math.sqrt(discriminant);

    // Two possible solutions
    double T1 = (-b + sqrt) / (2 * a);
    double T2 = (-b - sqrt) / (2 * a);

    // Choose the lower angle solution
    theta1 = Math.toDegrees(Math.atan(T1));
    theta2 = Math.toDegrees(Math.atan(T2));

    // Returns smallest angle
    if (isTheta1) {
      return theta1;
    } else {
      return theta2;
    }
    // return Math.min(theta1, theta2);
  }

  public double getTheta1() {
    return theta1;
  }

  public double getTheta2() {
    return theta2;
  }

  public double getBallVelFromFly(double flyVel) {
    double flyVelRadiansPerSec = flyVel * 6.28 / 60; // Converting from RPM to Rad/s
    double ballVel;
    ballVel =
        (flyWheelMOI * flyVelRadiansPerSec)
            / (flyWheelMOI + (massBall * radiusBallPath))
            * (104 / 2)
            / 10; // Final clause converts from rad/s to cm/s
    // Returns in Cm/S
    return ballVel;
  }

  public double getBallVelFromFinalFly(double flyVel) {
    // (Mass flywheel * flyVel)/(Mass flywheel + 2 mass ball)
    double finalVel =
        ((flyWheelMass * (flyVel / 60)) / (flyWheelMass + (2 * massBall)))
            * (104 / 2)
            / 1000; // Converts flyVel from Rpm to Rps, then multiplies by radius
    return finalVel;
  }

  public double getAccurateBallVelFromFinalFly(double flyVel) {
    // (Mass flywheel * flyVel)/(Mass flywheel + 2 mass ball)
    double finalVel =
        ((flyWheelMOI * (flyVel / 60)) / (flyWheelMOI + (massBall * 5.2 * 5.2)) * 5.2)
            / 100; // Converts flyVel from Rpm to Rps, then multiplies by radius
    return finalVel;
  }

  public double getFinalFlyVel(double flyVel) {
    // (Mass flywheel * flyVel)/(Mass flywheel + 2 mass ball)
    double finalVel =
        ((flyWheelMass * (flyVel))
            / (flyWheelMass + (2 * massBall))); // Converts flyVel to final velocity
    return finalVel;
  }

  public void hoodHome() {
    isHoming = true;
    if (motors.getTouchSensor() && isHoming) {
      motors.setHoodServoPower(-0.5);
    } else if (!motors.getTouchSensor() && isHoming) {
      motors.setHoodServoPower(0);
      motors.resetHoodEncoder();
      isHoming = false;
      motors.setHoodServoPower(0);
    }
  }

  public boolean hasHomed() {
    return isHoming;
  }

  /** --------- Stilt logic --------- */

  // Current min/max for testing min/max values
  double currentAnlogMax = 0;

  double currentAnalogMin = 1;
  // Experimental min/max values
  public double analogMin = 0.006;
  public double analogMax = 3.286;
  public double analogSpread = analogMax - analogMin;
  int inc; // For counting rotations
  double currentAnalog;
  double lastAnalog;
  double actualRotation;

  // For reading experimental min/max values of analog
  public double readCurrentMax() {
    if (motors.getStiltAnalog() > currentAnlogMax) {
      currentAnlogMax = motors.getStiltAnalog();
    }
    return currentAnlogMax;
  }

  public double readCurrentMin() {
    if (motors.getStiltAnalog() < currentAnalogMin) {
      currentAnalogMin = motors.getStiltAnalog();
    }
    return currentAnalogMin;
  }

  // Wrapping logic
  double delta;
  double threshold;
  double normalized;

  public double readMultipleRotationFromStart() {
    currentAnalog = motors.getStiltAnalog();

    delta = currentAnalog - lastAnalog;
    threshold = analogSpread * 0.4;

    if (delta > threshold) {
      inc--;
    } else if (delta < -threshold) {
      inc++;
    }

    lastAnalog = currentAnalog;

    normalized = currentAnalog - analogMin;
    actualRotation = normalized + (inc * analogSpread);

    return actualRotation * 360 / analogSpread;
  }

  double lastRunTime;
  double currentRunTime;
  double deltaTime;

  public double getDeltaTime(double runtime) {
    currentRunTime = runtime;
    deltaTime = currentRunTime - lastRunTime;
    lastRunTime = currentRunTime;

    return deltaTime;
  }

  double stiltAngleError;
  double stiltErrorPower;
  double stiltDerivative;
  double lastStiltAngleError;
  double stiltIntegral;
  // Divide all the constants by 360 to convert to a more understandable rotations-relative tuning
  // rather then inputing really small numbers for degrees-relative tuning
  public final double stiltKP = 2.1 / 360;
  public final double stiltKI = 0 / 360;
  public final double stiltKD =
      0.01 / 360; // Seems small but helps with ocilation, at least without the stilt at the moment.
  public final double stiltKF = 0;

  // Run to position function - seeks position so it doesn't sink
  public void runToStiltPosition(double pos, double runtime) {
    stiltAngleError = pos - readMultipleRotationFromStart();

    stiltDerivative = (stiltAngleError - lastStiltAngleError) / getDeltaTime(runtime);
    stiltIntegral += (stiltAngleError * getDeltaTime(runtime));
    stiltIntegral = Math.max(-0.1, Math.min(0.1, stiltIntegral));

    stiltErrorPower =
        (stiltAngleError * stiltKP)
            + (stiltDerivative * stiltKD)
            + (stiltIntegral * stiltKI)
            + stiltKF;
    lastStiltAngleError = stiltAngleError;

    motors.runTiltServo(Math.max(-0.9, Math.min(0.9, -stiltErrorPower)));
  }

  // To change position with driver input
  double seekPos;
  double tune = 5;

  public double changePos(double change) {
    seekPos += (change * tune);
    return seekPos;
  }

  double headingPower;
  double lastError;
  double headingDeltaTime;
  double headingLastRunTime;
  boolean pChange;
  boolean iChange;
  boolean dChange;

  int resetInc;
  int lastIncMode;
  int adjustedInc;
  int lastAdjustedInc;
  double kP;
  double kI;
  double kD;

  public void tunePID(int incMode, int inc) {
    if (incMode - lastIncMode != 0) {
      resetInc = inc;
    }
    adjustedInc = inc - resetInc;
    if (incMode % 3 == 0) {
      pChange = true;
      iChange = false;
      dChange = false;
    } else if (incMode % 3 == 1) {
      pChange = false;
      iChange = true;
      dChange = false;
    } else if (incMode % 3 == 2) {
      pChange = false;
      iChange = false;
      dChange = true;
    }
    if (pChange) {
      if (adjustedInc - lastAdjustedInc > 0) {
        kP += 1;
      } else if (adjustedInc - lastAdjustedInc < 0) {
        kP -= 1;
      }
      lastAdjustedInc = adjustedInc;
    } else if (iChange) {
      if (adjustedInc - lastAdjustedInc > 0) {
        kI += 1;
      } else if (adjustedInc - lastAdjustedInc < 0) {
        kI -= 1;
      }
      lastAdjustedInc = adjustedInc;
    } else if (dChange) {
      if (adjustedInc - lastAdjustedInc > 0) {
        kD += 1;
      } else if (adjustedInc - lastAdjustedInc < 0) {
        kD -= 1;
      }
      lastAdjustedInc = adjustedInc;
    }
    lastIncMode = incMode;
  }

  public double getKP() {
    return kP;
  }

  public double getkI() {
    return kI;
  }

  public double getkD() {
    return kD;
  }

  public boolean isPCHange() {
    return pChange;
  }

  public boolean isIChange() {
    return iChange;
  }

  public boolean isDChange() {
    return dChange;
  }

  public int getAdjustedInc() {
    return adjustedInc;
  }

  double termP;
  double termI;
  double termD;

  public double headingPID(double error, double runTime) {
    double adjustedError = error / 360;
    termP = adjustedError * kP;

    headingDeltaTime = runTime - headingLastRunTime;
    termD = ((adjustedError - lastError) / headingDeltaTime) * kD;

    termI += (adjustedError * headingDeltaTime) * kI;
    termI = Math.min(0.2, Math.max(-0.2, termI));

    headingPower = termP + termD + termI;

    lastError = adjustedError;
    headingLastRunTime = runTime;
    return headingPower;
  }
}
