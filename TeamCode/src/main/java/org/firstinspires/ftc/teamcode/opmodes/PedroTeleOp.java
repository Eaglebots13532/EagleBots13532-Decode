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
  private CarlHoodShoot hood;

  private boolean gateOpen = false;

  private double flywheelPower = 1400.0;
  boolean isHoming = false;
  boolean lastIntake = false;
  double seekHoodAngle;
  boolean isShooting = false;
  boolean lastB;
  boolean isIntaking = false;
  double flyVel = 3500;
  boolean lastDPadLeft = false;
  boolean lastDPadRight = false;

  @Override
  public void init() {
    gameDriver = new GameDriver(hardwareMap, telemetry);

    april = new AprilDriver(this);
    april.initAprilTag();

    hood = new CarlHoodShoot(gameDriver);
    /*
    inputStateMachine = new InputStateMachine(gamepad1, gamepad2);

    inputStateMachine.setListener(
        new InputStateMachine.StateListener() {
          // A button -- gamepad2: toggle gate
          @Override
          public void onTogglePrimary(int gamepad, boolean active) {

            if (gamepad == 2) {
              gateOpen = !gateOpen;
              if (gateOpen) {
                gameDriver.openGate();
              } else {
                gameDriver.closeGate();
              }
            }
          }

          // B button -- gamepad2: toggle intake
          @Override
          public void onToggleSecondary(int gamepad, boolean active) {
            if (gamepad == 2) {
              gameDriver.toggleIntake();
            }
          }

          // X button -- gamepad2: send chute home
          @Override
          public void onActionX(int gamepad) {
            if (gamepad == 2) {
              // autoRange = !autoRange;
            } // end if gamepad = 2
          } // end onActionX

          // Y button -- gamepad2: emergency stop chute
          @Override
          public void onActionY(int gamepad) {

            if (gamepad == 2) {}
          }

          // Dpad right -- gamepad2: extend chute
          @Override
          // onIncrementUp
          public void onIncrementUp(int gamepad) {
            if (gamepad == 2) {
              // chutePos += 2.0;
              // int encPulse = game.EncCntfrmRange(chutePos);
              // gameDriver.HoodPosition(encPulse);
            }
          }

          // Dpad left -- gamepad2: retract chute
          @Override
          // onIncrementDown
          public void onIncrementDown(int gamepad) {
            if (gamepad == 2) {
              // chutePos -= 2.0;
              // int encPulse = game.EncCntfrmRange(chutePos);
              // gameDriver.HoodPosition(encPulse);
            }
          }

          @Override
          public void on_D_Pad_Left(int gamepad) {
            // do nothing
          }

          @Override
          public void on_D_Pad_Right_Released(int gamepad) {
            if (gamepad == 2) {
              // gameDriver.Chute_Stop();
            }
          }

          @Override
          public void on_D_Pad_Left_Released(int gamepad) {
            if (gamepad == 2) {
              // gameDriver.Chute_Stop();
            }
          }

          // Dpad up
          @Override
          // on_D_Pad_Right
          public void on_D_Pad_Right(int gamepad) {
            if (gamepad == 2) {
              flywheelPower += 50.0;
              if (flywheelPower > 2200.0) {
                flywheelPower = 2100.0;
              }
              gameDriver.setLaunchVelocity(flywheelPower);
            }
          }
            //FlyWheel
          @Override
          public void onModifierLeft(int gamepad, float value) {
            if (gamepad == 2) {
              double InPwr = value;
              if (InPwr < 0.2) InPwr = 0.0;
              gameDriver.setIntakePower(InPwr);
            }
          }

          // Right trigger -- gamepad2: flywheel speed (proportional)
          @Override
          public void onModifierRight(int gamepad, float value) {
            if (gamepad == 2) {
              double inPwr = value;
              if (inPwr < .2) {
                inPwr = 0.0;
              }
              gameDriver.setLaunchPower(inPwr);
            }
          }
        });

     */

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

    // This is the normal version to use in the TeleOp
    if (!slowMode)
      follower.setTeleOpDrive(
          -gamepad1.left_stick_y,
          -gamepad1.left_stick_x,
          -gamepad1.right_stick_x,
          true // true = Robot Centric; false = Field Centric
          );

    // This is how it looks with slowMode on
    else
      follower.setTeleOpDrive(
          -gamepad1.left_stick_y * slowModeMultiplier,
          -gamepad1.left_stick_x * slowModeMultiplier,
          -gamepad1.right_stick_x * slowModeMultiplier,
          true // true = Robot Centric; false = Field Centric
          );

    /*
    Gamepad 2 Controls

    Left bumper - homing hood
    B - toggle intake on
    A - toggle intake off
    Y - toggle shooting sequence on (Intake and gate)
    X - toggle shooting sequence off (Intake and gate)

    Left joystick y - tilt position (Endgame)
     */
    if (gamepad2.b) {
      isIntaking = true;
    }
    if (gamepad2.a) {
      isIntaking = false;
    }
    if (isIntaking) {
      gameDriver.setIntakePower(1);
    } else if (!isIntaking) {
      gameDriver.setIntakePower(0);
    }

    // Toggle shooting sequence
    if (gamepad2.y) {
      isShooting = true;
    }
    if (gamepad2.x) {
      isShooting = false;
    }
    if (isShooting) {
      gameDriver.setGate(0.75);
      gameDriver.setIntakePower(1);
    }
    if (!isShooting) {
      gameDriver.setGate(0.5);
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
  }
}
