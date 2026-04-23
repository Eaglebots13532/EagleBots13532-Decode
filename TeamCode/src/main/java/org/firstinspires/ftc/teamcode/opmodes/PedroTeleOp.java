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
import org.firstinspires.ftc.teamcode.drivers.AprilDriver;
import org.firstinspires.ftc.teamcode.drivers.GameDriver;
import org.firstinspires.ftc.teamcode.drivers.odo.CarlOdometryExampleImplementation;
import org.firstinspires.ftc.teamcode.drivers.odo.WebCamCarlCoaxSwerve;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp(name = "Pedro Teleop")
public class PedroTeleOp extends OpMode {
  private Follower follower;

  private boolean slowMode = false;
  private double slowModeMultiplier = 0.5;

  public static Pose startingPose; // See ExampleAuto to understand how to use this

  private TelemetryManager telemetryM;

  private GameDriver gameDriver;
  private AprilDriver april;

  private InputStateMachine inputStateMachine;
  private CarlOdometryExampleImplementation odo;
  private CarlHoodShoot hood;
  private WebCamCarlCoaxSwerve webcam;

  boolean isHoodHoming = true;
  boolean isArmHoming = true;
  double seekHoodAngle;
  boolean isShooting = false;
  boolean isIntaking = false;
  double flyVel = 3700;
  boolean lastDPadLeft = false;
  boolean lastDPadRight = false;
  boolean autoAim = true;
  double lastCamDist = 1; // in meters
  boolean theta1;
  double checkHoodAngle;
  boolean isReady;
  double differenceFlyVel;
  boolean headingSeekMode = false;
  double yawError;
  double currentYaw;
  double yawSeek;
  double yawKP = 0.1;
  double finalYawValue;
  boolean distanceReady = false;
  boolean manualFly = true;
  boolean velReady;
  boolean endgame = false;
  boolean shooterSlow;
  boolean isBackTaking;

  @Override
  public void init() {
    gameDriver = new GameDriver(hardwareMap, telemetry);

    april = new AprilDriver(this);
    april.initAprilTag();

    odo = new CarlOdometryExampleImplementation(april);
    odo.init(hardwareMap);

    hood = new CarlHoodShoot(gameDriver);

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
    gameDriver.setGate(0.5);
  }

  @Override
  public void loop() {
    // Call this once per loop
    follower.update();
    telemetryM.update();
    odo.updatePos();

    // Commented out for debugging
    // inputStateMachine.captureInputs();
    // inputStateMachine.processInputs();

    telemetryM.debug("position", follower.getPose());
    telemetryM.debug("velocity", follower.getVelocity());

    /*
    Gamepad 1 Controls

    Left joystick - Forwards, backwards, strafe
    Right joystick - Yaw/rotation control


     */

    // Slow Mode
    if (gamepad1.rightBumperWasPressed()) {
      slowMode = !slowMode;
    }

    // For auto yaw seeking
    // yawError = currentYaw - yawSeek;
    /*
    if (gamepad1.dpad_up) {
      headingSeekMode = true;
    }
    if (gamepad1.dpad_down) {
      headingSeekMode = false;
    }


    if (headingSeekMode) {
      yawError = april.getActualBearing();
      finalYawValue = hood.headingPID(yawError, getRuntime());
      telemetry.addData("Bearing is:", april.getBearing());
      telemetry.addData("Actual Bearing is:", april.getActualBearing());
    }
     */

    // This is the normal version to use in the TeleOp
    if (!headingSeekMode && !slowMode)
      follower.setTeleOpDrive(
          -gamepad1.left_stick_y,
          -gamepad1.left_stick_x,
          -gamepad1.right_stick_x,
          true // true = Robot Centric; false = Field Centric
          );

    // This is how it looks with slowMode on
    else if (!headingSeekMode && slowMode)
      follower.setTeleOpDrive(
          -gamepad1.left_stick_y * slowModeMultiplier,
          -gamepad1.left_stick_x * slowModeMultiplier,
          -gamepad1.right_stick_x * slowModeMultiplier,
          true // true = Robot Centric; false = Field Centric
          );
    /*
    // Auto seeking
    else if (headingSeekMode)
      follower.setTeleOpDrive(
          -gamepad1.left_stick_y,
          -gamepad1.left_stick_x,
          finalYawValue,
          true // true = Robot Centric; false = Field Centric
          );

     */

    // Resets the hood odometry field positioning for testing
    if (gamepad1.xWasPressed()) {
      odo.resetFieldXY();
    }

    /*
    Gamepad 2 Controls

    Left bumper - homing hood
    A - toggle shooting sequence on (Intake and gate)
    B - toggle shooting sequence off (Intake and gate)
    Y - toggle intake on
    X - toggle intake off

    Left joystick y - tilt position (Endgame)
     */

    // Toggle Intake
    if (gamepad2.y) {
      isIntaking = true;
    }
    if (gamepad2.x) {
      isIntaking = false;
    }

    // Toggles shooting mode
    if (gamepad2.aWasPressed()) {
      isShooting = true;
      isReady = false;
    }
    if (gamepad2.bWasPressed()) {
      isShooting = false;
      isReady = false;
      isIntaking = false;
      distanceReady = false;
    }

    // Checks camera distance
    if (isShooting && !distanceReady) {
      lastCamDist = april.getRange() * 2.54 / 100; // Converts inches to Meters
      distanceReady = true;
    }

    // Checks to see if velocity is in range
    if (isShooting) {
      if (gameDriver.getFlyVelRPM() - flyVel > -300 && gameDriver.getFlyVelRPM() - flyVel < 300) {
        velReady = true;
      } else {
        velReady = false;
      }
      // Checks to see if we are ready
      if (velReady && distanceReady) {
        isReady = true;
      } else {
        isReady = false;
      }
    }

    // Checks to see if we are ready

    // Loads the balls when ready, turns the intake off but keeps gate open when not
    if (isShooting && isReady) {
      isIntaking = true;
      gameDriver.setGate(0.75);
    } else if (isShooting && !isReady) {
      isIntaking = false;
      gameDriver.setGate(0.5);
    }
    if (!isShooting) {
      gameDriver.setGate(0.5);
    }

    if (gamepad2.left_bumper) {
      isBackTaking = true;
    } else {
      isBackTaking = false;
    }

    if (isBackTaking) {
      isIntaking = false;
      gameDriver.setIntakePower(-1);
    }

    // Sets intake power
    if (isIntaking) {
      gameDriver.setIntakePower(1);
    } else if (!isIntaking) {
      gameDriver.setIntakePower(0);
    }

    /*
    // Manually changes hood angle
    if (gamepad2.dpad_up && !autoAim) {
      seekHoodAngle++;
    }
    if (gamepad2.dpad_down && !autoAim) {
      seekHoodAngle--;
    }


    // Toggles Auto Aim for Testing
    if (gamepad1.right_bumper) {
      autoAim = true;
    }
    if (gamepad1.left_bumper) {
      autoAim = false;
    }

     */

    // Auto aim
    if (autoAim) {
      checkHoodAngle =
          90
              - hood.getLaunchAngle(
                  lastCamDist,
                  1.15,
                  hood.getBallVelFromFly(hood.getFinalFlyVel(gameDriver.getFlyVelRPM())) / 100,
                  theta1);
      if (checkHoodAngle > 0 && checkHoodAngle < 90) {
        seekHoodAngle = checkHoodAngle;
      }
      telemetry.addData("We are looking at:", checkHoodAngle);
      telemetry.addData("Distance in Meters is:", lastCamDist);
      telemetry.addData("Final FlyVel is:", hood.getFinalFlyVel(gameDriver.getFlyVelRPM()));
    }
    gameDriver.setHoodServoPower(hood.getHoodServoPowerPID(seekHoodAngle, getRuntime()));

    // Hood homing
    if (isHoodHoming) {
      hood.hoodHome();
      isHoodHoming = hood.hasHomed();
    }

    // Arm homing
    if (gamepad2.right_bumper) {
      isArmHoming = true;
    }
    if (isArmHoming) {
      gameDriver.homeArm();
      isArmHoming = gameDriver.getArmHomeSensor();
    }
    // If arm isn't doing anything, set power to 0
    // gameDriver.manageArmPower();

    telemetry.addData("SeekHoodAngle is:", seekHoodAngle);
    telemetry.addData("Hood Angle is:", hood.getHoodAngle());
    telemetry.addData("Angle Error is:", hood.getAngleError());

    /** Endgame Code * */

    // Pivot arm down and up
    if (gamepad2.dpadDownWasPressed()) {
      endgame = true;
      flyVel = 0;
      gameDriver.setLaunchPower(0);
    } else if (gamepad2.dpadUpWasPressed()) {
      gameDriver.putArmUp();
      autoAim = true;
      endgame = false;
    }
    // Waits for shooter to slow down before allowing endgame
    if (gameDriver.getFlyVelRPM() < 1200) {
      shooterSlow = true;
    } else {
      shooterSlow = false;
    }
    // Enables Stilt and Lift control, only during the endgame (For some reason a while loop doesnt
    // work here)
    if (endgame && shooterSlow) {
      gameDriver.putArmDown();
      autoAim = false;
      seekHoodAngle = 0;
      gameDriver.setFlyVelRPM(0);
      endgame = true;
      gameDriver.setGate(0.5);
    }
    if (endgame) {
      // Stilt control - left joystick gamepad 2
      hood.runToStiltPosition(hood.changePos(-gamepad2.left_stick_y), getRuntime());
      gameDriver.powerLiftMotor(-gamepad2.right_stick_y);
    }

    // Flywheel Velocity control

    // Simple driver input controls for testing
    if (gamepad2.dpadRightWasPressed() && manualFly) {
      flyVel += 50;
    }
    if (gamepad2.dpadLeftWasPressed() && manualFly) {
      flyVel -= 50;
    }
    gameDriver.setFlyVelRPM(flyVel);

    telemetry.addData("FlyVel is:", flyVel);
    telemetry.addData("Current Vel is:", gameDriver.getFlyVelRPM());
    telemetry.addData("Arm Encoder is:", gameDriver.getArmPosition());

    telemetry.addData("Current FieldX is:", odo.getFieldX());
    telemetry.addData("Current FieldY is:", odo.getFieldY());

    telemetry.addData("Current Range is:", april.getRange());
    telemetry.addData("Current Bearing is:", april.getBearing());
    telemetry.addData("Current ActualBearing is:", april.getActualBearing());
    telemetry.addData("Is Blue April Tag:", april.getIsBlueAprilTag());
    telemetry.addData("Is Red April Tag:", april.getIsRedAprilTag());

    telemetry.addData("Distace ready", distanceReady);

    telemetry.addData("Touch Sensor:", gameDriver.getArmHomeSensor());

    // april.getRobotPoseFromTag()
  }
}
