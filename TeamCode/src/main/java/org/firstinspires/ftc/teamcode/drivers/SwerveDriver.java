// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.drivers;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import java.util.Optional;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drivers.odo.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.drivers.wpilib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.drivers.wpilib.math.controller.PIDController;
import org.firstinspires.ftc.teamcode.drivers.wpilib.util.Units;

/**
 * Standalone parallel-wheel swerve driver.
 *
 * <p>Two drive modes: - Field-centric swerve: wheels steer freely, translation is field-relative -
 * Tank drive: wheels lock forward, left/right joysticks control wheel speeds
 */
public class SwerveDriver {

  // --- Hardware ---
  private DcMotorEx[] driveMotors = new DcMotorEx[2]; // [0]=left, [1]=right
  private Servo[] steerServos = new Servo[2];
  private AnalogInput[] encoders = new AnalogInput[2];
  private GoBildaPinpointDriver pinpoint;
  private final Telemetry telemetry;

  // --- Physical constants ---
  private static final double wheelDiameterMm = 75.0;
  private static final double motorMaxRPM = 1150;
  private static final double gearRatio = 3;
  private static final double wheelBaseWidthMm = 355.5;

  // --- Motor encoder conversion ---
  private static final double ticksPerMotorRev = 28.0; // goBILDA 5203 series
  private static final double ticksPerWheelRev = ticksPerMotorRev * gearRatio;
  private static final double wheelCircumMeters = Math.PI * wheelDiameterMm / 1000.0;
  private static final double metersPerTick = wheelCircumMeters / ticksPerWheelRev;

  // --- Velocity feedback (one per wheel) ---
  private final PIDController[] velocityPIDs = {
    new PIDController(0.15, 0, 0), new PIDController(0.15, 0, 0)
  };

  // --- Derived constants ---
  private static final double wheelRadiusMeters = wheelDiameterMm / 1000.0 / 2.0;

  double scaleSpeed = 8.0;
  public final double maxSpeedMetersPerSec =
      scaleSpeed
          * Units.rotationsPerMinuteToRadiansPerSecond(motorMaxRPM / gearRatio)
          * wheelRadiusMeters;

  // kV maps m/s -> motor power [0..1].  power = speed * kV
  private final double kV = 1.0 / maxSpeedMetersPerSec;

  // Half the wheelbase in meters -- used for differential rotation
  private static final double halfWheelbaseMeters = wheelBaseWidthMm / 1000.0 / 2.0;

  public final double maxOmegaRadPerSec = maxSpeedMetersPerSec / halfWheelbaseMeters;

  // --- Per-wheel analog encoder offsets (calibrated so 0 deg = forward) ---
  private static final double leftEncoderOffsetDeg = -70.0;
  private static final double rightEncoderOffsetDeg = -82.0;
  private final Rotation2d[] encoderOffsets =
      new Rotation2d[] {
        Rotation2d.fromDegrees(leftEncoderOffsetDeg), Rotation2d.fromDegrees(rightEncoderOffsetDeg)
      };

  // --- Swerve state ---
  private Rotation2d lastTargetAngle = Rotation2d.kZero;
  private double lastTimeStamp;
  private final double[] lastErrorRad = new double[2];

  // --- Heading hold ---
  private final PIDController headingHoldPID = new PIDController(2.0, 0.5, 0.05);
  private Optional<Rotation2d> holdHeading = Optional.empty();

  // --- Tank drive: servo position that points wheels straight forward ---
  private static final double TANK_STEERING_CENTER = 0.5;

  public SwerveDriver(HardwareMap hardwareMap, Telemetry telemetry) {
    this.telemetry = telemetry;

    driveMotors[0] = (DcMotorEx) hardwareMap.dcMotor.get("LFM");
    driveMotors[1] = (DcMotorEx) hardwareMap.dcMotor.get("RFM");
    driveMotors[0].setDirection(DcMotorSimple.Direction.REVERSE);

    steerServos[0] = hardwareMap.servo.get("LFS");
    steerServos[1] = hardwareMap.servo.get("RFS");

    encoders[0] = hardwareMap.get(AnalogInput.class, "LFP");
    encoders[1] = hardwareMap.get(AnalogInput.class, "RFP");

    pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
    pinpoint.resetPosAndIMU();

    lastTimeStamp = System.nanoTime() / 1e9;

    // Heading PID wraps at +/- pi so it takes the shortest path
    headingHoldPID.enableContinuousInput(-Math.PI, Math.PI);
  }

  public void resetYaw() {
    pinpoint.setHeading(Rotation2d.kZero);
    holdHeading = Optional.empty();
  }

  // -----------------------------------------------------------------------
  // Tank drive -- wheels lock forward, direct left/right power
  // -----------------------------------------------------------------------

  public void tankDrive(double leftPower, double rightPower) {
    double drive = leftPower;
    double turn = rightPower;
    steerServos[0].setPosition(TANK_STEERING_CENTER);
    steerServos[1].setPosition(TANK_STEERING_CENTER);

    rightPower = drive + turn;
    leftPower = drive - turn;
    driveMotors[0].setPower(Range.clip(leftPower, -1.0, 1.0));
    driveMotors[1].setPower(Range.clip(rightPower, -1.0, 1.0));

    telemetry.addData("Drive Mode", "TANK");
    telemetry.addData("Left Power", "%.2f", leftPower);
    telemetry.addData("Right Power", "%.2f", rightPower);
  }

  // -----------------------------------------------------------------------
  // Field-centric swerve drive (exact copy of DC_Swerve_Drive logic)
  // -----------------------------------------------------------------------

  public void swerveDrive(
      double fieldXVelMetersPerSec, double fieldYVelMetersPerSec, double chassisOmegaRadPerSec) {
    pinpoint.update();

    // --- Step 1: Field-to-robot frame rotation ---
    var inverseYaw = pinpoint.getYaw().unaryMinus();
    double chassisXVel =
        fieldXVelMetersPerSec * inverseYaw.getCos() - fieldYVelMetersPerSec * inverseYaw.getSin();
    double chassisYVel =
        fieldXVelMetersPerSec * inverseYaw.getSin() + fieldYVelMetersPerSec * inverseYaw.getCos();

    // --- Step 2: Clamp movement speed to motor limits ---
    double speed = Math.hypot(chassisXVel, chassisYVel);
    if (speed > maxSpeedMetersPerSec) {
      double scale = maxSpeedMetersPerSec / speed;
      chassisXVel *= scale;
      chassisYVel *= scale;
      speed = maxSpeedMetersPerSec;
    }

    // --- Step 3: Compute shared steering angle ---
    Rotation2d targetAngle;

    if (speed > 0.01) {
      targetAngle = new Rotation2d(chassisXVel, chassisYVel);
      lastTargetAngle = targetAngle;
    } else if (Math.abs(chassisOmegaRadPerSec) > 0.01) {
      // Rotation only -- point wheels forward for differential spin
      targetAngle = Rotation2d.kZero;
      lastTargetAngle = targetAngle;
    } else {
      // Nothing commanded -- hold everything
      targetAngle = lastTargetAngle;
    }

    // --- Step 4: Heading hold ---
    double effectiveOmega;
    double currentYawRad = pinpoint.getYaw().getRadians();

    if (Math.abs(chassisOmegaRadPerSec) > 0.01) {
      effectiveOmega = chassisOmegaRadPerSec;
      holdHeading = Optional.of(pinpoint.getYaw());
    } else if (speed > 0.01) {
      if (holdHeading.isEmpty()) {
        holdHeading = Optional.of(pinpoint.getYaw());
      }
      double headingCorrection =
          headingHoldPID.calculate(currentYawRad, holdHeading.get().getRadians());
      effectiveOmega =
          Math.max(-maxOmegaRadPerSec * 0.5, Math.min(maxOmegaRadPerSec * 0.5, headingCorrection));
    } else {
      effectiveOmega = 0;
      holdHeading = Optional.of(pinpoint.getYaw());
    }

    // --- Step 5: Compute per-wheel drive power ---
    double basePower = speed * kV;
    double rotationDelta = effectiveOmega * halfWheelbaseMeters * kV;

    double[] drivePowers = {
      basePower - rotationDelta, // left wheel
      basePower + rotationDelta // right wheel
    };

    double currentTime = System.nanoTime() / 1e9;
    double dt = currentTime - lastTimeStamp;

    // --- Telemetry ---
    telemetry.addLine("Yaw: " + pinpoint.getYaw());
    telemetry.addLine("Target angle: " + targetAngle);
    if (holdHeading.isPresent()) {
      telemetry.addLine("Hold heading: " + holdHeading.get());
    }
    telemetry.addLine("Effective omega: " + effectiveOmega);
    telemetry.update();

    // --- Step 6: Per-wheel steering PID and coordinated flip ---
    Rotation2d[] currentAngles = new Rotation2d[2];
    Rotation2d[] angleErrors = new Rotation2d[2];
    for (int i = 0; i < 2; i++) {
      currentAngles[i] =
          Rotation2d.fromRotations(-encoders[i].getVoltage() / encoders[i].getMaxVoltage())
              .plus(encoderOffsets[i]);
      angleErrors[i] = targetAngle.minus(currentAngles[i]);
    }

    boolean flipMotors =
        Math.abs(angleErrors[0].getDegrees()) > 90 && Math.abs(angleErrors[1].getDegrees()) > 90;

    for (int i = 0; i < 2; i++) {
      var angleError = angleErrors[i];
      double power = drivePowers[i];

      if (flipMotors) {
        power *= -1;
        angleError = targetAngle.plus(Rotation2d.k180deg).minus(currentAngles[i]);
      }

      // Cosine scaling: reduce drive power while wheel is mid-turn
      power *= angleError.getCos();

      // Closed-loop velocity: feedforward + PID correction from motor encoder
      double targetVel = power * maxSpeedMetersPerSec;
      double actualVel = driveMotors[i].getVelocity() * metersPerTick;
      double correction = velocityPIDs[i].calculate(actualVel, targetVel);
      driveMotors[i].setPower(power + correction);

      // Steering PID -> servo position
      double steeringAngle = calculateSteerPID(angleError, i, dt) / 2 + 0.5;
      steerServos[i].setPosition(steeringAngle);
    }

    lastTimeStamp = currentTime;
  }

  // -----------------------------------------------------------------------
  // Steering PID
  // -----------------------------------------------------------------------

  private double calculateSteerPID(Rotation2d angleError, int i, double dt) {
    double errorRad = angleError.getRadians();

    // Deadband: ignore encoder noise to prevent kS from chattering
    if (Math.abs(errorRad) < Math.toRadians(2.0)) {
      lastErrorRad[i] = 0;
      return 0.0;
    }

    double kP = 1.25 / (Math.PI / 2); // Full output at 90 deg error
    double kD = 0.01;
    double kS = 0.03; // Static friction compensation

    double proportional = errorRad * kP;
    double derivative = kD * (errorRad - lastErrorRad[i]) / dt;
    lastErrorRad[i] = errorRad;

    double output = proportional + derivative;
    return output + kS * Math.signum(output);
  }
}
