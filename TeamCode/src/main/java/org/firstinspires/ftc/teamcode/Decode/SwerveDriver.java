// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Decode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.Optional;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Decode.odo.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Decode.wpilib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.Decode.wpilib.util.Units;

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
  private static final double WHEEL_DIAMETER_MM = 75.0;
  private static final double MOTOR_MAX_RPM = 1150;
  private static final double GEAR_RATIO = 3;
  private static final double WHEEL_BASE_WIDTH_MM = 355.5;

  // --- Derived constants ---
  private static final double WHEEL_RADIUS_METERS = WHEEL_DIAMETER_MM / 1000.0 / 2.0;
  private static final double HALF_WHEELBASE_METERS = WHEEL_BASE_WIDTH_MM / 1000.0 / 2.0;

  public final double maxSpeedMetersPerSec =
      Units.rotationsPerMinuteToRadiansPerSecond(MOTOR_MAX_RPM / GEAR_RATIO) * WHEEL_RADIUS_METERS;
  public final double maxOmegaRadPerSec = maxSpeedMetersPerSec / HALF_WHEELBASE_METERS;

  private final double kV = 1.0 / maxSpeedMetersPerSec;

  // --- Encoder offsets (calibrated so 0 deg = forward) ---
  private static final double LEFT_ENCODER_OFFSET_DEG = -70.0;
  private static final double RIGHT_ENCODER_OFFSET_DEG = -82.0;
  private final Rotation2d[] encoderOffsets =
      new Rotation2d[] {
        Rotation2d.fromDegrees(LEFT_ENCODER_OFFSET_DEG),
        Rotation2d.fromDegrees(RIGHT_ENCODER_OFFSET_DEG)
      };

  // --- Swerve state ---
  private Rotation2d lastTargetAngle = Rotation2d.kZero;
  private double lastTimeStamp;
  private final double[] lastErrorRad = new double[2];
  private Optional<Rotation2d> startingYawAngle = Optional.empty();
  private Optional<Boolean> leadingWheelEast = Optional.empty();

  // --- Tank drive: servo position that points wheels straight forward ---
  // This is the servo value (0.0-1.0) where PID output = 0 => centered = 0.5
  // Adjust if wheels don't point straight at 0.5
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
  }

  public void resetYaw() {
    pinpoint.setHeading(Rotation2d.kZero);
  }

  // -----------------------------------------------------------------------
  // Tank drive -- wheels lock forward, direct left/right power
  // -----------------------------------------------------------------------

  /**
   * Tank drive mode. Both wheels are locked pointing straight forward. Each joystick directly
   * controls its corresponding wheel speed.
   *
   * @param leftPower left wheel power (-1.0 to 1.0)
   * @param rightPower right wheel power (-1.0 to 1.0)
   */
  public void tankDrive(double leftPower, double rightPower) {
    // Lock both wheels to straight forward
    steerServos[0].setPosition(TANK_STEERING_CENTER);
    steerServos[1].setPosition(TANK_STEERING_CENTER);

    // Direct power control
    driveMotors[0].setPower(leftPower);
    driveMotors[1].setPower(rightPower);

    telemetry.addData("Drive Mode", "TANK");
    telemetry.addData("Left Power", "%.2f", leftPower);
    telemetry.addData("Right Power", "%.2f", rightPower);
  }

  // -----------------------------------------------------------------------
  // Field-centric swerve drive
  // -----------------------------------------------------------------------

  /**
   * Field-centric swerve drive. Both wheels steer to the same angle (parallel constraint). Rotation
   * is achieved via differential wheel speed.
   *
   * @param xVelMetersPerSec lateral velocity (field frame)
   * @param yVelMetersPerSec forward velocity (field frame)
   * @param omegaRadPerSec desired rotational rate (positive = CCW)
   */
  public void swerveDrive(double xVelMetersPerSec, double yVelMetersPerSec, double omegaRadPerSec) {
    pinpoint.update();

    // --- Step 1: Field-to-robot frame rotation ---
    var inverseYaw = pinpoint.getYaw().unaryMinus();
    double chassisXVel =
        xVelMetersPerSec * inverseYaw.getCos() - yVelMetersPerSec * inverseYaw.getSin();
    double chassisYVel =
        xVelMetersPerSec * inverseYaw.getSin() + yVelMetersPerSec * inverseYaw.getCos();

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
    double requestedTranslationAngle = Math.atan2(yVelMetersPerSec, xVelMetersPerSec);

    Rotation2d yawRotationalDrift = new Rotation2d();
    if (startingYawAngle.isPresent()) {
      yawRotationalDrift = inverseYaw.minus(startingYawAngle.get());
    }

    if (speed > 0.01) {
      targetAngle = new Rotation2d(chassisXVel, chassisYVel);
      lastTargetAngle = targetAngle;

      if (startingYawAngle.isEmpty()) {
        Rotation2d leftJoyRotation = new Rotation2d(xVelMetersPerSec, yVelMetersPerSec);
        leftJoyRotation = leftJoyRotation.minus(Rotation2d.fromRadians(requestedTranslationAngle));
        startingYawAngle = Optional.of(leftJoyRotation);
      }
    } else if (Math.abs(omegaRadPerSec) > 0.01) {
      targetAngle = Rotation2d.kZero;
      startingYawAngle = Optional.empty();
    } else {
      targetAngle = lastTargetAngle;
      startingYawAngle = Optional.empty();
    }

    // --- Step 4: Compute per-wheel drive power ---
    double basePower = speed * kV;
    double rotationDelta = omegaRadPerSec * HALF_WHEELBASE_METERS * kV;

    double rotationDriftPowerCompensationRight = 0.0;
    double rotationDriftPowerCompensationLeft = 0.0;

    if (startingYawAngle.isPresent()) {
      if (yawRotationalDrift.getRadians() > 0.01) {
        rotationDriftPowerCompensationRight = 0.5;
        rotationDriftPowerCompensationLeft = 0.0;
      } else if (yawRotationalDrift.getRadians() < -0.01) {
        rotationDriftPowerCompensationRight = 0.0;
        rotationDriftPowerCompensationLeft = 0.5;
      }

      if (requestedTranslationAngle <= 0.5 && requestedTranslationAngle >= -0.5) {
        // Heading north -- use differential power compensation
      } else if (requestedTranslationAngle >= 3.0 || requestedTranslationAngle <= -3.0) {
        // Heading south -- use differential power compensation
      } else if (requestedTranslationAngle > 0.0) {
        // Heading west -- use rudder correction instead
        rotationDriftPowerCompensationRight = 0.0;
        rotationDriftPowerCompensationLeft = 0.0;
        leadingWheelEast = Optional.of(false);
      } else {
        // Heading east -- use rudder correction instead
        rotationDriftPowerCompensationRight = 0.0;
        rotationDriftPowerCompensationLeft = 0.0;
        leadingWheelEast = Optional.of(true);
      }
    }

    double[] drivePowers = {
      basePower - rotationDelta + rotationDriftPowerCompensationLeft, // left
      basePower + rotationDelta + rotationDriftPowerCompensationRight // right
    };

    double currentTime = System.nanoTime() / 1e9;
    double dt = currentTime - lastTimeStamp;

    // --- Step 5: Per-wheel steering PID and flip optimization ---
    for (int i = 0; i < 2; i++) {
      var currentAngle =
          Rotation2d.fromRotations(-encoders[i].getVoltage() / encoders[i].getMaxVoltage())
              .plus(encoderOffsets[i]);

      var angleError = targetAngle.minus(currentAngle);
      double power = drivePowers[i];

      // Flip optimization: reverse motor instead of turning 180 deg
      if (Math.abs(angleError.getDegrees()) > 90) {
        power *= -1;
        angleError = targetAngle.plus(Rotation2d.k180deg).minus(currentAngle);
      }

      // Cosine scaling: reduce power while wheel is mid-turn
      power *= angleError.getCos();

      driveMotors[i].setPower(power);

      // Steering PID -> servo position
      double steeringAngle = calculateSteerPID(angleError, i, dt) / 2 + 0.5;

      double rudderCorrection = 0.0;
      if (leadingWheelEast.isPresent()) {
        double candidateRudder = 0.0;
        if (yawRotationalDrift.getRadians() > 0.01) {
          candidateRudder = -0.1;
        } else if (yawRotationalDrift.getRadians() < -0.01) {
          candidateRudder = 0.1;
        }
        if (leadingWheelEast.get()) {
          if (i == 0) rudderCorrection = candidateRudder;
        } else {
          if (i == 1) rudderCorrection = candidateRudder;
        }
      }

      steerServos[i].setPosition(steeringAngle + rudderCorrection);
    }

    lastTimeStamp = currentTime;

    // --- Telemetry ---
    telemetry.addData("Drive Mode", "FIELD-CENTRIC");
    telemetry.addData("Speed", "%.2f m/s", speed);
    telemetry.addData("Target Angle", targetAngle);
  }

  // -----------------------------------------------------------------------
  // Steering PID
  // -----------------------------------------------------------------------

  private double calculateSteerPID(Rotation2d angleError, int i, double dt) {
    double errorRad = angleError.getRadians();
    double kP = 1.25 / (Math.PI / 2); // full output at 90 deg error
    double kD = 0.01;
    double kS = 0.03; // static friction compensation

    double proportional = errorRad * kP;
    double derivative = kD * (errorRad - lastErrorRad[i]) / dt;
    lastErrorRad[i] = errorRad;

    double output = proportional + derivative;
    return output + kS * Math.signum(output);
  }
}
