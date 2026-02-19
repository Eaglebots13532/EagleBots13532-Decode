// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.teamcode.drivers.DriveManager;
import org.firstinspires.ftc.teamcode.drivers.GameDriver;
import org.firstinspires.ftc.teamcode.StateMachine.InputStateMachine;
import org.firstinspires.ftc.teamcode.chute.ChuteDriver;
import org.firstinspires.ftc.teamcode.chute.FtcPotentiometer;

@TeleOp(name = "Decode Teleop")
public class DecodeTeleop extends LinearOpMode {

  private static final double CHUTE_STEP = 3.0;
  private static final double POT_WRAP_AMOUNT = 6.16;

  private boolean chuteInputsLocked = false;
  private boolean gateOpen = false;

  private double flywheelPower = 0.0;

  @Override
  public void runOpMode() {
    // --- Subsystems ---
    DriveManager driveManager = new DriveManager(hardwareMap, telemetry);
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
            chuteInputsLocked = false;
            // telemetry.addLine("Chute reached: " +  managerposition);
          }

          @Override
          public void onHomeComplete() {
            chuteInputsLocked = false;
            telemetry.addLine("Chute homed");
          }

          @Override
          public void onHomeTimeout() {
            chuteInputsLocked = false;
            telemetry.addLine("WARNING: Chute home timed out");
          }

          @Override
          public void onStopped() {
            chuteInputsLocked = false;
            telemetry.addLine("Chute stopped");
          }
        });

    // --- Input callbacks ---
    sm.setListener(
        new InputStateMachine.StateListener() {
          // A button -- gamepad2: toggle gate
          @Override
          public void onTogglePrimary(int gamepad, boolean active) {
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
            if (gamepad == 2 && !chuteInputsLocked) {
              chuteInputsLocked = true;
              chute.goHome();
            }
          }

          // Y button -- gamepad2: emergency stop chute
          @Override
          public void onActionY(int gamepad) {
            if (gamepad == 2) {
              chute.stop();
            }
          }

          // Dpad up -- gamepad2: extend chute
          @Override
          public void onIncrementUp(int gamepad) {
            if (gamepad == 2 && !chuteInputsLocked) {
              chuteInputsLocked = true;
              chute.goToPosition(chute.getPosition() + CHUTE_STEP);
            }
          }

          // Dpad down -- gamepad2: retract chute
          @Override
          public void onIncrementDown(int gamepad) {
            if (gamepad == 2 && !chuteInputsLocked) {
              chuteInputsLocked = true;
              double target = chute.getPosition() - CHUTE_STEP;
              if (target < 0.001) {
                chute.goHome();
              } else {
                chute.goToPosition(target);
              }
            }
          }

          // Dpad left -- gamepad1: toggle drive mode
          @Override
          public void onCycleLeft(int gamepad) {
            if (gamepad == 1) {
              driveManager.toggleMode();
            } else if (gamepad == 2) {
              flywheelPower -= 0.1;
              if (flywheelPower < 0.1) {
                flywheelPower = 0.0;
              }
              game.setLaunchPower(flywheelPower);
            }
          }

          @Override
          public void onCycleRight(int gamepad) {
            if (gamepad == 2) {
              flywheelPower += 0.1;
              if (flywheelPower > 9.0) {
                flywheelPower = 0.95;
              }
              game.setLaunchPower(flywheelPower);
            }
          }

          @Override
          public void onModifierLeft(int gamepad, float value) {}

          // Right trigger -- gamepad2: flywheel speed (proportional)
          @Override
          public void onModifierRight(int gamepad, float value) {
            if (gamepad == 2) {
              game.setLaunchPower(value);
            }
          }
        });

    telemetry.addLine("Initialized -- waiting for start");
    telemetry.addData("Drive Mode", driveManager.getMode());
    telemetry.update();

    waitForStart();

    /* --- Home the chute before entering main loop ---
     *

    telemetry.addLine("Homing chute...");
    telemetry.update();
    chute.goHome();
    while (opModeIsActive() && chute.isBusy()) {
      chute.update();
      telemetry.update();
      sleep(20);
    }
    */

    // --- Main loop ---
    while (opModeIsActive()) {
      // Discrete button events
      sm.captureInputs();
      sm.processState();

      // Gamepad1: drive (polled)
      driveManager.drive(
          -gamepad1.left_stick_x,
          -gamepad1.left_stick_y,
          -gamepad1.right_stick_x,
          -gamepad1.right_stick_y);

      // Gamepad2: arm and tilt (polled, continuous)
      game.setArmPower(-gamepad2.right_stick_y);
      game.setTilt((-gamepad2.left_stick_y + 1.0) / 2.0); // map -1..1 to 0..1

      // Subsystem updates
      chute.update();
      game.updateTelemetry();
      telemetry.update();
      sleep(20);
    }
  }
}
