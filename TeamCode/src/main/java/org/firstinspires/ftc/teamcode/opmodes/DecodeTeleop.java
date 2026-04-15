// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.StateMachine.InputStateMachine;
import org.firstinspires.ftc.teamcode.drivers.AprilDriver;
import org.firstinspires.ftc.teamcode.drivers.DriveManager;
import org.firstinspires.ftc.teamcode.drivers.GameDriver;
import org.firstinspires.ftc.teamcode.drivers.wpilib.interpolation.InterpolatingDoubleTreeMap;

@TeleOp(name = "Decode Teleop")
public class DecodeTeleop extends LinearOpMode {

  private static double chutePos = 2.0;
  private boolean gateOpen = false;
  private boolean autoRange = false;

  private double flywheelPower = 1400.0;

  @Override
  public void runOpMode() {
    // --- Subsystems ---
    AprilDriver april = new AprilDriver(this);
    april.initAprilTag(); // initialize camera to read april tags
    DriveManager driveManager = new DriveManager(this, hardwareMap, telemetry);
    GameDriver game = new GameDriver(hardwareMap, telemetry, this);

    // --- Input ---
    InputStateMachine sm = new InputStateMachine(gamepad1, gamepad2);

    // --- Input callbacks ---
    sm.setListener(
        new InputStateMachine.StateListener() {
          // A button -- gamepad2: toggle gate
          @Override
          public void onTogglePrimary(int gamepad, boolean active) {
            if (gamepad == 1) {
              driveManager.resetYaw();
            }
            if (gamepad == 2) {
              gateOpen = !gateOpen;
              if (gateOpen) {
                game.openGate();
              } else {
                game.closeGate();
              }
            }
          }

          // B button -- gamepad2: toggle intake
          @Override
          public void onToggleSecondary(int gamepad, boolean active) {
            if (gamepad == 2) {
              game.toggleIntake();
            }
          }

          // X button -- gamepad2: send chute home
          @Override
          public void onActionX(int gamepad) {
            if (gamepad == 2) {
              autoRange = !autoRange;
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
              chutePos += 2.0;
              int encPulse = game.EncCntfrmRange(chutePos);
              game.HoodPosition(encPulse);
            }
          }

          // Dpad left -- gamepad2: retract chute
          @Override
          // onIncrementDown
          public void onIncrementDown(int gamepad) {
            if (gamepad == 2) {
              chutePos -= 2.0;
              int encPulse = game.EncCntfrmRange(chutePos);
              game.HoodPosition(encPulse);
            }
          }

          @Override
          public void on_D_Pad_Right_Released(int gamepad) {
            if (gamepad == 2) {
              // game.Chute_Stop();
            }
          }

          @Override
          public void on_D_Pad_Left_Released(int gamepad) {
            if (gamepad == 2) {
              // game.Chute_Stop();
            }
          }

          // Dpad down -- gamepad1: toggle drive mode
          @Override
          // on_D_Pad_Left
          public void on_D_Pad_Left(int gamepad) {
            if (gamepad == 1) {
              driveManager.toggleMode();
            } else if (gamepad == 2) {
              flywheelPower -= 50.0;
              if (flywheelPower < 1400.0) {
                flywheelPower = 1400.0;
              }
              game.setLaunchVelocity(flywheelPower);
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
              game.setLaunchVelocity(flywheelPower);
            }
          }

          @Override
          public void onModifierLeft(int gamepad, float value) {
            if (gamepad == 2) {
              double InPwr = value;
              if (InPwr < 0.2) InPwr = 0.0;
              game.setIntakePower(InPwr);
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
              game.setLaunchPower(inPwr);
            }
          }
        });

    telemetry.addLine("Initialized -- waiting for start");
    telemetry.addData("Drive Mode", driveManager.getMode());
    telemetry.update();

    waitForStart();
    // hood starts in home position After auto~ hood is sent home
    // if (game.homed()) {
    //  game.GotoHome();
    // }
    // game.setHome(); // sets hood encoder home encoder count
    // --- Main loop ---
    while (opModeIsActive()) {
      // -----------------------------------------
      // Discrete button events
      // -----------------------------------------
      sm.captureInputs();
      sm.processInputs();

      InterpolatingDoubleTreeMap distanceToFlywheelVelocity = new InterpolatingDoubleTreeMap();
      distanceToFlywheelVelocity.put(35.0, 1450.0);
      distanceToFlywheelVelocity.put(79.0, 1600.0);
      distanceToFlywheelVelocity.put(126.0, 1700.0);
      distanceToFlywheelVelocity.put(130.0, 1800.0);
      distanceToFlywheelVelocity.put(160.0, 1900.0);
      distanceToFlywheelVelocity.put(273.0, 2100.0);

      // -----------------------------------------
      // April tag and chute/flywheel auto adjust
      // -----------------------------------------
      if (autoRange) {
        // Load/refresh April tag information into AprilDriver members
        april.getAprilTag();
        telemetry.addData("April range", april.getRange());
        telemetry.addData("April tag", april.getMetaId());

        // Determine if a tag is within view of the camera
        // Retrieve values from recently refreshed AprilDriver members
        double range = april.getRange();
        int tag = april.getMetaId();
        boolean istag = tag == 20 || tag == 24;
        if (istag) {
          double flyvelocity = distanceToFlywheelVelocity.get(range);
          game.setLaunchVelocity(flyvelocity);
          // check hood position
          // chute.goToPosition(distanceToHoodMap.get(range));
        } else {
          // Tage not in visible range of camera
          //
          // The robot is close for launching set default 1500
          game.setLaunchVelocity(1500);
        }
      } // End autoRange check
      if (gamepad2.rightBumperWasPressed()) game.setLaunchVelocity(0.0);

      // -----------------------------------------
      // Joystick control for robot movement
      // -----------------------------------------
      // Gamepad1: drive (polled)
      driveManager.drive(
          gamepad1.left_stick_x,
          -gamepad1.left_stick_y,
          gamepad1.right_stick_x,
          -gamepad1.right_stick_y);

      // Gamepad2: arm and tilt (polled, continuous)
      game.setArmPower(-gamepad2.right_stick_y);
      // game.setTilt((-gamepad2.left_stick_y + 1.0) / 2.0); // map -1..1 to 0..1
      // Commented out because needed to update gamedriver to be combatable with carlhoodshoot as
      // tilt servo is now in continous instead of servo mode
      // Subsystem updates
      game.updateTelemetry();
      telemetry.addData("Auto Range ", autoRange ? "On" : "Off");
      telemetry.update();
      sleep(20);
    }
  }
}
