// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.StateMachine.InputStateMachine;
import org.firstinspires.ftc.teamcode.drivers.AprilDriver;
import org.firstinspires.ftc.teamcode.drivers.DriveManager;
import org.firstinspires.ftc.teamcode.drivers.GameDriver;
import org.firstinspires.ftc.teamcode.drivers.chute.FtcPotentiometer;
import org.firstinspires.ftc.teamcode.drivers.wpilib.interpolation.InterpolatingDoubleTreeMap;

@TeleOp(name = "Decode Teleop")
public class DecodeTeleop extends LinearOpMode {

  private static final double CHUTE_STEP = 2.0;
  private static final double POT_WRAP_AMOUNT = 6.16;
  private boolean gateOpen = false;
  private boolean autoRange = false;

  private double flywheelPower = 1400.0;
  GoBildaPrismDriver prism;

  @Override
  public void runOpMode() {
    // --- Subsystems ---
    prism = hardwareMap.get(GoBildaPrismDriver.class, "prism"); // not used rn
    AprilDriver april = new AprilDriver(this);
    april.initAprilTag(); // initialize camera to read april tags
    DriveManager driveManager = new DriveManager(this, hardwareMap, telemetry);
    GameDriver game = new GameDriver(hardwareMap, telemetry);

    CRServo chuteMotor = hardwareMap.get(CRServo.class, "chute");
    AnalogInput chutePot = hardwareMap.get(AnalogInput.class, "CP");
    FtcPotentiometer pot = new FtcPotentiometer(chutePot, POT_WRAP_AMOUNT);
    ChuteDriver chute = new ChuteDriver(chuteMotor, pot, telemetry);

    // --- Input ---
    InputStateMachine sm = new InputStateMachine(gamepad1, gamepad2);

    // --- Chute completion callbacks ---
    chute.setListener(
        new ChuteDriver.ChuteListener() {
          @Override
          public void onTargetReached(double position) {
            // telemetry.addLine("Chute reached: " +  managerposition);
          }

          @Override
          public void onHomeComplete() {
            telemetry.addLine("Chute homed");
          }

          @Override
          public void onHomeTimeout() {
            telemetry.addLine("WARNING: Chute home timed out");
          }

          @Override
          public void onStopped() {
            telemetry.addLine("Chute stopped");
          }
        });

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
            if (gamepad == 2) {
              chute.stop();
            }
          }

          // Dpad right -- gamepad2: extend chute
          @Override
          // onIncrementUp
          public void onIncrementUp(int gamepad) {
            if (gamepad == 2) {
              chute.extend();
            }
          }

          // Dpad left -- gamepad2: retract chute
          @Override
          // onIncrementDown
          public void onIncrementDown(int gamepad) {
            if (gamepad == 2) {
              chute.retract();
            }
          }

          @Override
          public void on_D_Pad_Right_Released(int gamepad) {
            if (gamepad == 2) {
              chute.stop();
            }
          }

          @Override
          public void on_D_Pad_Left_Released(int gamepad) {
            if (gamepad == 2) {
              chute.stop();
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

    InterpolatingDoubleTreeMap distanceToHoodMap = new InterpolatingDoubleTreeMap();
    distanceToHoodMap.put(296.0, 10.5);
    distanceToHoodMap.put(160.0, 9.5);
    distanceToHoodMap.put(130.0, 8.5);
    distanceToHoodMap.put(126.0, 8.0);
    distanceToHoodMap.put(79.0, 4.0);
    distanceToHoodMap.put(43.0, 2.0);

    InterpolatingDoubleTreeMap distanceToFlywheelVelocity = new InterpolatingDoubleTreeMap();
    distanceToFlywheelVelocity.put(35.0, 1450.0);
    distanceToFlywheelVelocity.put(79.0, 1600.0);
    distanceToFlywheelVelocity.put(126.0, 1700.0);
    distanceToFlywheelVelocity.put(130.0, 1800.0);
    distanceToFlywheelVelocity.put(160.0, 1900.0);
    distanceToFlywheelVelocity.put(273.0, 2100.0);

    waitForStart();

    // --- Main loop ---
    while (opModeIsActive()) {
      // -----------------------------------------
      // Discrete button events
      // -----------------------------------------
      sm.captureInputs();
      sm.processInputs();

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
      game.setTilt((-gamepad2.left_stick_y + 1.0) / 2.0); // map -1..1 to 0..1

      // Subsystem updates
      chute.update();
      game.updateTelemetry();
      telemetry.addData("Auto Range ", autoRange ? "On" : "Off");
      telemetry.update();
      sleep(20);
    }
  }
}
