// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Decode;

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

  DC_Swerve_Drive(LinearOpMode opmode) {
    myOp = opmode;
  }

  // Hardware
  private DcMotorEx[] driveMotors = new DcMotorEx[2]; // [0]=left, [1]=right
  private Servo[] steerServos = new Servo[2];
  private AnalogInput[] encoders = new AnalogInput[2];
  private GoBildaPinpointDriver pinpoint;

  // Physical constants
  private static final double wheelDiameterMm = 96.0;
  private static final double motorMaxRPM = 1150;
  private static final double gearRatio = 3;
  private static final double wheelBaseWidthMm = 355.5;
  public static final double mEnc = 537.7; // PPR

  // Derived: max wheel speed in m/s
  //   wheelRadiusMeters = (diameter_mm / 1000) / 2
  //   wheelAngularVel = RPM_at_wheel * 2*pi / 60
  //   linearVel = wheelAngularVel * wheelRadius
  private static final double wheelRadiusMeters = wheelDiameterMm / 1000.0 / 2.0;
  double maxSpeedMetersPerSec =
      Units.rotationsPerMinuteToRadiansPerSecond(motorMaxRPM / gearRatio) * wheelRadiusMeters;

  // kV maps m/s -> motor power [0..1].  power = speed * kV
  double kV = 1.0 / maxSpeedMetersPerSec;

  // Half the wheelbase in meters -- used for differential rotation
  private static final double halfWheelbaseMeters = wheelBaseWidthMm / 1000.0 / 2.0;

  // Max rotation rate (rad/s) when both wheels are fully dedicated to turning
  // (one full forward, one full reverse, no translation)
  public double maxOmegaRadPerSec = maxSpeedMetersPerSec / halfWheelbaseMeters;

  // Per-wheel analog encoder offsets (calibrated so 0 deg = forward).
  // Adjust these until both wheels point straight ahead when the joystick is
  // pushed forward. If a wheel aims too far left, increase its value.
  private static final double leftEncoderOffsetDeg = -2.5;
  private static final double rightEncoderOffsetDeg = -5.0;
  Rotation2d[] encoderOffsets =
      new Rotation2d[] {
        Rotation2d.fromDegrees(leftEncoderOffsetDeg),
        Rotation2d.fromDegrees(rightEncoderOffsetDeg)
      };

  // Tracks the last steering direction so wheels hold position when joystick is released
  private Rotation2d lastTargetAngle = Rotation2d.kZero;
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

  /**
   * Field-centric parallel-wheel swerve drive.
   *
   * <p>Both wheels always steer to the same angle (parallel constraint). Rotation is achieved via
   * differential wheel speed, like a tank drive that can be oriented in any direction.
   *
   * @param fieldXVelMetersPerSec Left joystick X -- field-relative lateral velocity
   * @param fieldYVelMetersPerSec Left joystick Y -- field-relative forward velocity
   * @param chassisOmegaRadPerSec Right joystick X -- desired rotational rate
   */
  public void fieldRelativeDrive(
      double fieldXVelMetersPerSec, double fieldYVelMetersPerSec, double chassisOmegaRadPerSec) {
    pinpoint.update();

    // --- Step 1: Field-to-robot frame rotation ---
    // "Yaw" is the robot's rotation as seen from above -- imagine looking straight
    // down at the field. 0 deg = the direction the robot faced at startup. Turning
    // left increases yaw, turning right decreases it (counter-clockwise positive).
    //
    // The problem: the joystick gives us a direction relative to the FIELD (push up
    // = move away from the driver, always). But the wheels are attached to the ROBOT,
    // which may be rotated. We need to convert field directions into robot directions.
    //
    // We do this by rotating the joystick vector by the NEGATIVE (inverse) of the
    // robot's yaw. Example: robot is turned 90 deg left. Driver pushes joystick "up"
    // (field-forward). From the robot's perspective, that's actually to its RIGHT.
    // Rotating by -90 deg converts field-forward into robot-right. That's what the
    // matrix multiply below does for any angle.
    //
    // The math (standard 2D rotation matrix with angle = -yaw):
    //   [cos(-t)  -sin(-t)] [vx_field]   =  [vx_robot]
    //   [sin(-t)   cos(-t)] [vy_field]      [vy_robot]
    var inverseYaw = pinpoint.getYaw().unaryMinus();
    double chassisXVel =
        fieldXVelMetersPerSec * inverseYaw.getCos()
            - fieldYVelMetersPerSec * inverseYaw.getSin();
    double chassisYVel =
        fieldXVelMetersPerSec * inverseYaw.getSin()
            + fieldYVelMetersPerSec * inverseYaw.getCos();

    // --- Step 2: Clamp movement speed to motor limits ---
    // "speed" is how fast the robot is sliding across the field (ignoring rotation).
    // The joystick could ask for more than the motors can deliver -- e.g. pushing
    // into a corner asks for full forward AND full sideways at once. If that happens,
    // scale the velocity down so it stays within the motor's max while keeping the
    // same direction.
    double speed = Math.hypot(chassisXVel, chassisYVel);
    if (speed > maxSpeedMetersPerSec) {
      double scale = maxSpeedMetersPerSec / speed;
      chassisXVel *= scale;
      chassisYVel *= scale;
      speed = maxSpeedMetersPerSec;
    }

    // --- Step 3: Compute shared steering angle ---
    // Both wheels point in the direction the robot-frame velocity vector is going.
    // atan2(y, x) gives us the angle of that vector -- this is the direction the
    // wheels need to face. If the joystick is released (no translation), we hold
    // the last angle so the wheels don't snap to some default.
    Rotation2d targetAngle;
    if (speed > 0.01) {
      // atan2(y, x) gives the angle of the velocity vector
      targetAngle = new Rotation2d(chassisXVel, chassisYVel);
      lastTargetAngle = targetAngle;
    } else {
      targetAngle = lastTargetAngle;
    }

    // --- Step 4: Compute per-wheel drive power ---
    // Base power comes from how far the joystick is pushed (translational speed).
    // To rotate, we make one wheel go faster and the other slower -- just like a
    // tank/skid-steer robot turns by spinning its sides at different speeds.
    // Positive omega -> robot turns CCW -> right wheel faster, left wheel slower.
    //   omega = (v_right - v_left) / wheelbase
    //   so v_left  = v_base - omega * wheelbase/2
    //      v_right = v_base + omega * wheelbase/2
    double basePower = speed * kV;
    double rotationDelta = chassisOmegaRadPerSec * halfWheelbaseMeters * kV;

    double[] drivePowers = {
      basePower - rotationDelta, // left wheel
      basePower + rotationDelta // right wheel
    };

    double currentTime = System.nanoTime() / 1e9;
    double dt = currentTime - lastTimeStamp;

    // --- Step 5: Per-wheel steering PID and flip optimization ---
    for (int i = 0; i < 2; i++) {
      // Read current wheel angle from the analog encoder (maps voltage to one full rotation)
      var currentAngle =
          Rotation2d.fromRotations(-encoders[i].getVoltage() / encoders[i].getMaxVoltage())
              .plus(encoderOffsets[i]);

      var angleError = targetAngle.minus(currentAngle);
      double power = drivePowers[i];

      // Flip optimization: suppose the wheel currently points right and we want it
      // to point left. That's 180 deg of turning. But we can instead keep pointing
      // right and just reverse the motor -- same effect, zero steering needed. We
      // apply this whenever the error exceeds 90 deg: flip the motor and target the
      // opposite direction, which is always closer.
      if (Math.abs(angleError.getDegrees()) > 90) {
        power *= -1;
        angleError = targetAngle.plus(Rotation2d.k180deg).minus(currentAngle);
      }

      // Cosine scaling: while the wheel is still turning toward the target angle,
      // we reduce drive power proportionally. cos(0) = 1 (full power when aligned),
      // cos(90) = 0 (no power when perpendicular). This prevents the robot from
      // lurching sideways while the wheel is still mid-turn.
      power *= angleError.getCos();

      driveMotors[i].setPower(power);
      // The steering PID outputs a value from -1 to 1 (full left to full right).
      // Servos expect 0 to 1, so we map: servo_pos = (pid_output / 2) + 0.5
      //   pid = -1 -> servo = 0.0 (full one direction)
      //   pid =  0 -> servo = 0.5 (centered)
      //   pid =  1 -> servo = 1.0 (full other direction)
      steerServos[i].setPosition(calculateSteerPID(angleError, i, dt) / 2 + 0.5);
    }

    lastTimeStamp = currentTime;
  }

  // --- Steering PID ---
  private final double[] lastErrorRad = new double[2];

  private double calculateSteerPID(Rotation2d angleError, int i, double dt) {
    double errorRad = angleError.getRadians();
    double kP = 1.25 / (Math.PI / 2); // Full output at 90 deg error
    double kD = 0.01;
    double kS = 0.03; // Static friction compensation

    double proportional = errorRad * kP;
    double derivative = kD * (errorRad - lastErrorRad[i]) / dt;
    lastErrorRad[i] = errorRad;

    double output = proportional + derivative;
    // Add a small constant in the direction of output to overcome static friction
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