// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Decode; // Copyright (c) 2024-2025 FTC 13532

// All rights reserved.

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.teamcode.Decode.wpilib.util.Units;

public class DC_Swerve_Drive {
  private LinearOpMode myOp = null;

  // Define a constructor that allows the OpMode to pass a reference to itself.
  DC_Swerve_Drive(LinearOpMode opmode) {
    myOp = opmode;
  }

  // *** - *** -
  // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
  private DcMotorEx[] driveMotors = new DcMotorEx[2];
  private CRServo[] steerServos = new CRServo[2];
  private AnalogInput[] encoders = new AnalogInput[2];

  public IMU imu;
  // ---
  // Swerve chassis constants in inches (to be done)
  private static final double wheelDiameterMm = 96.0; // mm
  private static final double wCir = wheelDiameterMm * 3.141592; // 301.593 wheel circumference
  public static final double mEnc = 537.7; // PPR
  private static final double motorMaxRPM = 1150;
  private static final double gearRatio = 3; // gear ratio
  private static final double wheelBaseWidthMm = 355.5;

  double maxSpeedMetersPerSec =
      Units.rotationsPerMinuteToRadiansPerSecond(motorMaxRPM / gearRatio)
          * wheelDiameterMm
          * 1000
          / 2;
  double kV = 1.0 / maxSpeedMetersPerSec;

  double[] modulesXPosMeters =
      new double[] {wheelBaseWidthMm * 1000 / 2, -wheelBaseWidthMm * 1000 / 2};
  double[] modulesYPosMeters = new double[] {0, 0};

  // The max rotational rate
  double drivebaseRadiusMeters =
      Math.max(
          Math.hypot(modulesXPosMeters[0], modulesYPosMeters[0]),
          Math.hypot(modulesXPosMeters[1], modulesYPosMeters[1]));
  double maxOmegaRadPerSec = maxSpeedMetersPerSec / drivebaseRadiusMeters;

  public void init() {
    driveMotors[0] = (DcMotorEx) myOp.hardwareMap.dcMotor.get("LFM");
    driveMotors[1] = (DcMotorEx) myOp.hardwareMap.dcMotor.get("RFM");
    driveMotors[1].setDirection(DcMotorSimple.Direction.REVERSE);
    steerServos[0] = myOp.hardwareMap.crservo.get("LFS");
    steerServos[1] = myOp.hardwareMap.crservo.get("RFS");
    encoders[0] = myOp.hardwareMap.get(AnalogInput.class, "LFP");
    encoders[1] = myOp.hardwareMap.get(AnalogInput.class, "RFP");
    imu = myOp.hardwareMap.get(IMU.class, "imu");
  }

  public void drive(
      double fieldXVelMetersPerSec, double fieldYVelMetersPerSec, double chassisOmegaRadPerSec) {
    // Convert field oriented velocities to robot oriented velocities
    double robotYawRad = imu.getRobotYawPitchRollAngles().getYaw();
    double chassisXVelMetersPerSec =
        fieldXVelMetersPerSec * Math.cos(-robotYawRad)
            - fieldYVelMetersPerSec * Math.sin(-robotYawRad);
    double chassisYVelMetersPerSec =
        fieldXVelMetersPerSec * Math.sin(-robotYawRad)
            + fieldYVelMetersPerSec * Math.cos(-robotYawRad);

    for (int i = 0; i < 2; i++) {
      // Calculate the X and Y velocities of each module
      double targetXVelMetersPerSec =
          chassisXVelMetersPerSec - chassisOmegaRadPerSec * modulesYPosMeters[i];
      double targetYVelMetersPerSec =
          chassisYVelMetersPerSec + chassisOmegaRadPerSec * modulesXPosMeters[i];

      // Calculate the velocity vector magnitude and angle
      double targetVelocityMetersPerSec =
          Math.hypot(targetXVelMetersPerSec, targetYVelMetersPerSec);
      double targetAngleRad = Math.atan2(targetYVelMetersPerSec, targetXVelMetersPerSec);

      // Calculate motor power and target steer angle
      double driveMotorPower = targetVelocityMetersPerSec * kV;
      double currentAngleRad = encoders[i].getVoltage() / encoders[i].getMaxVoltage() * 2 * Math.PI;
      // If the steer angle delta is too large, flip around the direction of the target speed to
      // avoid
      // turning too far
      double angleErrorRad = addAngles(targetAngleRad, -currentAngleRad);
      if (Math.abs(angleErrorRad) > Math.PI / 2) {
        driveMotorPower *= -1;
        angleErrorRad = addAngles(addAngles(targetAngleRad, Math.PI), -currentAngleRad);
      }

      driveMotorPower *= Math.cos(angleErrorRad);

      myOp.telemetry.addData("Wheel " + i + " driveMotorPower", driveMotorPower);
      myOp.telemetry.addData("Wheel " + i + " angleErrorRad", angleErrorRad);

      // Drive the motor and the steer PID here
      // ...
      driveMotors[i].setPower(driveMotorPower);
      steerServos[i].setPower(angleErrorRad * 1);
    }
  }

  private double addAngles(double angle1Rad, double angle2Rad) {
    return Math.atan2(
        Math.cos(angle1Rad) * Math.sin(angle2Rad) + Math.sin(angle1Rad) * Math.cos(angle2Rad),
        Math.cos(angle1Rad) * Math.cos(angle2Rad) - Math.sin(angle1Rad) * Math.sin(angle2Rad));
  }
}
