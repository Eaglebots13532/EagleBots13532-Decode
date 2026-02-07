// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Decode; // Copyright (c) 2024-2025 FTC 13532

// All rights reserved.

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Decode.odo.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Decode.wpilib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.Decode.wpilib.math.controller.PIDController;
import org.firstinspires.ftc.teamcode.Decode.wpilib.util.Units;

public class DC_Swerve_Drive {
  private final LinearOpMode myOp;

  // Define a constructor that allows the OpMode to pass a reference to itself.
  DC_Swerve_Drive(LinearOpMode opmode) {
    myOp = opmode;
  }

  // *** - *** -
  // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
  private DcMotorEx[] driveMotors = new DcMotorEx[2];
  private Servo[] steerServos = new Servo[2];
  private AnalogInput[] encoders = new AnalogInput[2];
  private GoBildaPinpointDriver pinpoint;

  private double[] candidateSteerAmounts = new double[2];
  private double[] candidatePowerAmounts = new double[2];

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

  double[] modulesXPosMeters = new double[] {0, 0};
  double[] modulesYPosMeters =
      new double[] {wheelBaseWidthMm * 1000 / 2, -wheelBaseWidthMm * 1000 / 2};
  Rotation2d[] encoderOffsets =
      new Rotation2d[] {Rotation2d.fromDegrees(-2.5), Rotation2d.fromDegrees(-5)};

  // The max rotational rate
  double drivebaseRadiusMeters =
      Math.max(
          Math.hypot(modulesXPosMeters[0], modulesYPosMeters[0]),
          Math.hypot(modulesXPosMeters[1], modulesYPosMeters[1]));
  double maxOmegaRadPerSec = maxSpeedMetersPerSec / drivebaseRadiusMeters;

  private double lastTimeStamp;

  public void init() {
    driveMotors[0] = (DcMotorEx) myOp.hardwareMap.dcMotor.get("LFM");
    driveMotors[1] = (DcMotorEx) myOp.hardwareMap.dcMotor.get("RFM");
    driveMotors[0].setDirection(DcMotorSimple.Direction.REVERSE);
    steerServos[0] = myOp.hardwareMap.servo.get("LFS");
    steerServos[1] = myOp.hardwareMap.servo.get("RFS");
    encoders[0] = myOp.hardwareMap.get(AnalogInput.class, "LFP");
    encoders[1] = myOp.hardwareMap.get(AnalogInput.class, "RFP");
    pinpoint = myOp.hardwareMap.get(GoBildaPinpointDriver.class, "odo");
    lastTimeStamp = System.nanoTime() / 1e9;
  }

  public void fieldRelativeDrive(
      double fieldXVelMetersPerSec, double fieldYVelMetersPerSec, double chassisOmegaRadPerSec) {
    pinpoint.update();
    // Convert field oriented velocities to robot oriented velocities
    var robotYaw = pinpoint.getYaw();
    var inverseRobotYaw = pinpoint.getYaw().unaryMinus();
    double chassisXVelMetersPerSec =
        fieldXVelMetersPerSec * inverseRobotYaw.getCos()
            - fieldYVelMetersPerSec * inverseRobotYaw.getSin();
    double chassisYVelMetersPerSec =
        fieldXVelMetersPerSec * inverseRobotYaw.getSin()
            + fieldYVelMetersPerSec * inverseRobotYaw.getCos();
    var currentChassisOmega = pinpoint.getYawVelocityRadPerSec();
    chassisOmegaRadPerSec += 1 * (chassisOmegaRadPerSec - currentChassisOmega);

    // myOp.telemetry.addData("Gyro angle", robotYaw.getDegrees());
    // myOp.telemetry.addData("Gyro omega", currentChassisOmega);

    var translationalMagnitude = Math.hypot(chassisXVelMetersPerSec, chassisYVelMetersPerSec);
    if (translationalMagnitude > maxSpeedMetersPerSec) {
      chassisXVelMetersPerSec *= (translationalMagnitude / maxSpeedMetersPerSec);
      chassisYVelMetersPerSec *= (translationalMagnitude / maxSpeedMetersPerSec);
    }

    double currentTime = System.nanoTime() / 1e9;
    double dt = currentTime - lastTimeStamp;

    // Determine individual wheel steering angle and wheel power
    for (int i = 0; i < 2; i++) {
      // Calculate the X and Y velocities of each module
      double targetXVelMetersPerSec =
          chassisXVelMetersPerSec - chassisOmegaRadPerSec * modulesYPosMeters[i];
      double targetYVelMetersPerSec =
          chassisYVelMetersPerSec + chassisOmegaRadPerSec * modulesXPosMeters[i];

      // Calculate the velocity vector magnitude and angle
      double targetVelocityMetersPerSec =
          Math.hypot(targetXVelMetersPerSec, targetYVelMetersPerSec);
      var targetAngle = new Rotation2d(targetXVelMetersPerSec, targetYVelMetersPerSec);

      // Calculate motor power and target steer angle
      double driveMotorPower = targetVelocityMetersPerSec * kV;
      var currentAngle =
          Rotation2d.fromRotations(-encoders[i].getVoltage() / encoders[i].getMaxVoltage())
              .plus(encoderOffsets[i]);
      // If the steer angle delta is too large, flip around the direction of the target speed to
      // avoid turning too far
      var angleError = targetAngle.minus(currentAngle);
      if (Math.abs(angleError.getDegrees()) > 90) {
        driveMotorPower *= -1;
        targetAngle = targetAngle.plus(Rotation2d.k180deg);
        angleError = targetAngle.minus(currentAngle);
      }

      driveMotorPower *= angleError.getCos();

      // myOp.telemetry.addData("Wheel " + i + " driveMotorPower", driveMotorPower);
      // myOp.telemetry.addData("Wheel " + i + " targetAngle", targetAngle.getDegrees());
      // myOp.telemetry.addData("Wheel " + i + " currentAngle", currentAngle.getDegrees());
      // myOp.telemetry.addData("Wheel " + i + " angleError", angleError.getDegrees());

      candidatePowerAmounts[i] = driveMotorPower;
      candidateSteerAmounts[i] = calculateSteerPID(angleError, i, dt) / 2 + .5;

      // The following code is disabled so we can later lock the wheels to one another
      // Drive the motor and the steer PID here
      // driveMotors[i].setPower(driveMotorPower);
      // steerServos[i].setPosition(calculateSteerPID(angleError, i, dt) / 2 + .5);
    }

    // For now, allow the first wheel to the be master and have the other lock to it
    driveMotors[0].setPower(candidatePowerAmounts[0]);
    steerServos[0].setPosition(candidateSteerAmounts[0]);
    driveMotors[1].setPower(candidatePowerAmounts[0]);
    steerServos[1].setPosition(candidateSteerAmounts[0]);

    lastTimeStamp = currentTime;
  }

  private final double[] lastErrorRad = new double[2];

  private double calculateSteerPID(Rotation2d angleError, int i, double dt) {
    double errorRad = angleError.getRadians();
    double kP = 1.25 / (Math.PI / 2);
    double kD = 0.01;
    double kS = .03;
    double proportional = errorRad * kP;
    double derivative = kD * (errorRad - lastErrorRad[i]) / dt;
    // myOp.telemetry.addData("Wheel " + i + " proportional", proportional);
    // myOp.telemetry.addData("Wheel " + i + " derivative", derivative);
    lastErrorRad[i] = errorRad;
    var output = proportional + derivative;
    return output + kS * Math.signum(output);
  }

  public void resetYaw() {
    pinpoint.setHeading(Rotation2d.kZero);
  }

  private final PIDController xController = new PIDController(1, 0, 0);
  private final PIDController yController = new PIDController(1, 0, 0);
  private final PIDController yawController = new PIDController(1, 0, 0);

  public void pidToPose(double x, double y, double yawRad) {
    fieldRelativeDrive(
        xController.calculate(pinpoint.getXPosMeters(), x),
        yController.calculate(pinpoint.getYPosMeters(), y),
        yawController.calculate(pinpoint.getYaw().getRadians(), yawRad));
  }
}
