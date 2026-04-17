// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Decode.CarlHoodShoot;
import org.firstinspires.ftc.teamcode.StateMachine.InputStateMachine;
import org.firstinspires.ftc.teamcode.drivers.GameDriver;
import org.firstinspires.ftc.teamcode.drivers.odo.CarlOdometryExampleImplementation;
import org.firstinspires.ftc.teamcode.drivers.odo.WebCamCarlCoaxSwerve;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp(name = "CarlTestYawSeekPedro Teleop")
public class CarlTestYawSeekPedro extends OpMode {
  private Follower follower;

  private boolean slowMode = false;
  private double slowModeMultiplier = 0.5;

  public static Pose startingPose; // See ExampleAuto to understand how to use this

  private TelemetryManager telemetryM;

  private GameDriver gameDriver;

  private InputStateMachine inputStateMachine;
  private CarlHoodShoot hood;
  private WebCamCarlCoaxSwerve webcam;
  private CarlOdometryExampleImplementation odo;

  private boolean gateOpen = false;

  private double flywheelPower = 1400.0;
  boolean isHoming = false;
  boolean lastIntake = false;
  double seekHoodAngle;
  boolean isShooting = false;
  boolean lastB;
  boolean isIntaking = false;
  double flyVel = 0;
  boolean lastDPadLeft = false;
  boolean lastDPadRight = false;
  boolean seekMode = false;
  double seekYaw;
  double currentYaw;
  double changeYaw;
  private static final double yawAcceleration = 0.2;
  Double bearing;
  double lastTime;
  boolean coarseHeading = false;
  boolean fineHeading = false;
  boolean coarseHeadingDone = false;
  boolean fineHeadingDone = false;
  boolean headingReady = false;
  boolean isRedTeam = true;
  double aprilTagYaw;
  boolean shootingReady = false;
  int teamAprilTag;

  @Override
  public void init() {
    gameDriver = new GameDriver(hardwareMap, telemetry);

    hood = new CarlHoodShoot(gameDriver);

    webcam = new WebCamCarlCoaxSwerve();
    webcam.init(hardwareMap);

    odo = new CarlOdometryExampleImplementation(webcam);
    odo.init(hardwareMap);

    follower = Constants.createFollower(hardwareMap);
    follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
    follower.update();
    telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
  }

  @Override
  public void start() {
    // The parameter controls whether the Follower should use break mode on the motors (using it is
    // recommended).
    // In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in
    // Constant.java (for Mecanum)
    // If you don't pass anything in, it uses the default (false)
    follower.startTeleopDrive();
  }

  @Override
  public void loop() {
    // Call this once per loop
    follower.update();
    telemetryM.update();
    odo.accumulateFieldPos();

    // See's if red or blue team is selected (Blue is just not red)
    if (gamepad1.left_bumper) {
      isRedTeam = true;
      teamAprilTag = 24;
    }
    if (gamepad1.right_bumper) {
      isRedTeam = false;
      teamAprilTag = 20;
    }

    // Updates Yaw
    currentYaw = odo.getFieldYaw();

    if (isRedTeam) {
      aprilTagYaw = 30;
    } else {
      aprilTagYaw = -30;
    }

    telemetryM.debug("position", follower.getPose());
    telemetryM.debug("velocity", follower.getVelocity());

    /**
     * Gamepad 1 Controls
     *
     * <p>Left joystick - Forwards, backwards, strafe Right joystick - Yaw/rotation control
     */
    /*
    // Slow Mode
    if (gamepad1.rightBumperWasPressed()) {
      slowMode = !slowMode;
    }

     */
    /*
    // Seeking to a heading mode
    if (gamepad1.right_bumper) {
      seekMode = true;
    }
    if (gamepad1.left_bumper) {
      seekMode = false;
    }

     */

    if (seekMode && coarseHeading) {
      changeYaw = (seekYaw - currentYaw) * yawAcceleration;
    } else if (seekMode && fineHeading) {
      bearing = webcam.getAprilBearing(teamAprilTag);
    }

    if (fineHeading && bearing != null) {
      changeYaw = bearing * yawAcceleration;
    } else {
      changeYaw = 0; // or keep last value
    }
    // This is the normal version to use in the TeleOp
    if (!slowMode && !seekMode) {
      follower.setTeleOpDrive(
          -gamepad1.left_stick_y,
          -gamepad1.left_stick_x,
          -gamepad1.right_stick_x,
          true); // true = Robot Centric; false = Field Centric
    }
    // This is how it looks with slowMode on
    else if (slowMode && !seekMode) {
      follower.setTeleOpDrive(
          -gamepad1.left_stick_y * slowModeMultiplier,
          -gamepad1.left_stick_x * slowModeMultiplier,
          -gamepad1.right_stick_x * slowModeMultiplier,
          true // true = Robot Centric; false = Field Centric
          );
    }

    // Seek mode without slow mode
    else if (!slowMode && seekMode)
      follower.setTeleOpDrive(
          -gamepad1.left_stick_y,
          -gamepad1.left_stick_x,
          changeYaw,
          true // true = Robot Centric; false = Field Centric
          );

    // Seek mode with slow mode
    else if (slowMode && seekMode)
      follower.setTeleOpDrive(
          -gamepad1.left_stick_y * slowModeMultiplier,
          -gamepad1.left_stick_x * slowModeMultiplier,
          changeYaw * slowModeMultiplier,
          true // true = Robot Centric; false = Field Centric
          );

    /**
     * Gamepad 2 Controls
     *
     * <p>Left bumper - homing hood B - toggle intake on A - toggle intake off Y - toggle shooting
     * sequence on (Heading adjustment (Automatic, coarse with odometry then fine with camera), hood
     * angle check, then gate and intake) X - toggle shooting sequence off (Intake and gate, turn
     * off auto heading adjustment)
     *
     * <p>Left joystick y - tilt position (Endgame)
     */
    // Intake toggle
    if (gamepad2.b) {
      isIntaking = true;
    }
    if (gamepad2.a) {
      isIntaking = false;
    }

    // Toggle shooting sequence
    if (gamepad2.y) {
      isShooting = true;
      seekMode = true;
    }
    if (gamepad2.x) {
      isShooting = false;
      seekMode = false;
    }

    // Drive shooting sequence

    // Heading adjustment
    if (isShooting && !headingReady) {
      // Sees if coarse heading is complete
      if (coarseHeading && (changeYaw > -5 || changeYaw < 5)) {
        coarseHeading = false;
        coarseHeadingDone = true;
        fineHeading = true;
      }
      // Sees if camera based(Fine) heading is done
      if (fineHeading && (bearing > -2 || bearing < 2)) {
        fineHeading = false;
        fineHeadingDone = true;
      }

      // Adjusts heading using odometry
      if (coarseHeading) {
        seekYaw = aprilTagYaw;
      }

      if (fineHeadingDone && coarseHeadingDone) {
        headingReady = true;
      }
    }

    // Checks to see if hood is ready and

    if (!isShooting) {
      gameDriver.setGate(0.5);
      gameDriver.setIntakePower(0);
    }
    // Set intake power
    if (isIntaking) {
      gameDriver.setIntakePower(1);
    } else if (!isIntaking) {
      gameDriver.setIntakePower(0);
    }

    // Manually changes hood angle
    if (gamepad2.dpad_up) {
      seekHoodAngle++;
    }
    if (gamepad2.dpad_down) {
      seekHoodAngle--;
    }
    gameDriver.setHoodServoPower(hood.getHoodServoPowerPID(seekHoodAngle, getRuntime()));

    // Hood homing
    if (gamepad2.left_bumper) {
      isHoming = true;
    }
    if (isHoming) {
      hood.hoodHome();
      isHoming = hood.hasHomed();
    }

    telemetry.addData("SeekHoodAngle is:", seekHoodAngle);
    telemetry.addData("Hood Angle is:", hood.getHoodAngle());
    telemetry.addData("Angle Error is:", hood.getAngleError());

    // Stilt control - left joystick gamepad 2
    hood.runToStiltPosition(hood.changePos(-gamepad2.left_stick_y), getRuntime());

    // Flywheel Velocity control

    // Simple driver input controls for testing
    if (gamepad2.dpad_left && !lastDPadLeft) {
      flyVel += 200;
    }
    if (gamepad2.dpad_right && !lastDPadRight) {
      flyVel -= 200;
    }

    gameDriver.setFlyVelRPM(flyVel);

    telemetry.addData("FlyVel is:", flyVel);
    telemetry.addData("Current Vel is:", gameDriver.getFlyVelRPM());

    lastDPadLeft = gamepad2.dpad_left;
    lastDPadRight = gamepad2.dpad_right;

    telemetry.addData("Apriltag Bearing is:", webcam.getAprilBearing(teamAprilTag));

    telemetry.addData("Delta Time is:", getRuntime() - lastTime);
    lastTime = getRuntime();

    telemetry.addData("FieldX is:", odo.getFieldX());
    telemetry.addData("FieldY is:", odo.getFieldY());
    telemetry.addData("FieldYaw is:", odo.getFieldYaw());

    telemetry.addData("RobotX is:", odo.getRobotX());
    telemetry.addData("RobotY is:", odo.getRobotY());

    telemetry.addData("Pinpoint X is:", odo.getPinpointX());
    telemetry.addData("Pinpoint Y is:", odo.getPinpointY());
    telemetry.addData("Pinpoint Yaw is:", odo.getPinpointYaw());

    telemetry.addData("Delta X is:", odo.getDeltaRobotX());
    telemetry.addData("Delta Y is:", odo.getDeltaRobotY());

    telemetry.addData("Last X is:", odo.getLastPosX());
    telemetry.addData("Last Y is:", odo.getLastPosY());

    telemetry.addData("Delta Field X is:", odo.getDeltaFieldX());
    telemetry.addData("Delta Field Y is:", odo.getDeltaFieldY());

    // Shooting button test

  }
}
