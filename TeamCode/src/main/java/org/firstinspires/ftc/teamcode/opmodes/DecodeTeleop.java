// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.teamcode.Decode.DriveManager;
import org.firstinspires.ftc.teamcode.StateMachine.InputStateMachine;
import org.firstinspires.ftc.teamcode.chute.ChuteDriver;
import org.firstinspires.ftc.teamcode.chute.FtcPotentiometer;

@TeleOp(name = "Decode Teleop")
public class DecodeTeleop extends LinearOpMode {

  private static final double CHUTE_STEP = 3.0;
  private static final double POT_WRAP_AMOUNT = 6.16;

  private boolean chuteInputsLocked = false;

  @Override
  public void runOpMode() {
    // --- Subsystems ---
    DriveManager driveManager = new DriveManager(hardwareMap, telemetry);

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
            telemetry.addLine("Chute reached: " + position);
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
          @Override
          public void onTogglePrimary(int gamepad, boolean active) {}

          @Override
          public void onToggleSecondary(int gamepad, boolean active) {}

          @Override
          public void onActionX(int gamepad) {
            if (gamepad == 2 && !chuteInputsLocked) {
              chuteInputsLocked = true;
              chute.goHome();
            }
          }

          @Override
          public void onActionY(int gamepad) {
            if (gamepad == 2) {
              chute.stop();
            }
          }

          @Override
          public void onIncrementUp(int gamepad) {
            if (gamepad == 2 && !chuteInputsLocked) {
              chuteInputsLocked = true;
              chute.goToPosition(chute.getPosition() + CHUTE_STEP);
            }
          }

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

          @Override
          public void onCycleLeft(int gamepad) {
            if (gamepad == 1) {
              driveManager.toggleMode();
            }
          }

          @Override
          public void onCycleRight(int gamepad) {}

          @Override
          public void onModifierLeft(int gamepad, float value) {}

          @Override
          public void onModifierRight(int gamepad, float value) {}
        });

    telemetry.addLine("Initialized -- waiting for start");
    telemetry.addData("Drive Mode", driveManager.getMode());
    telemetry.update();

    waitForStart();

    // --- Home the chute before entering main loop ---
    telemetry.addLine("Homing chute...");
    telemetry.update();
    chute.goHome();
    while (opModeIsActive() && chute.isBusy()) {
      chute.update();
      telemetry.update();
      sleep(20);
    }

    // --- Main loop ---
    while (opModeIsActive()) {
      // Discrete button events
      sm.captureInputs();
      sm.processState();

      // Joysticks polled directly -- all four axes passed,
      // DriveManager picks which ones matter based on mode
      driveManager.drive(
          -gamepad1.left_stick_x,
          -gamepad1.left_stick_y,
          -gamepad1.right_stick_x,
          -gamepad1.right_stick_y);

      // Subsystem updates
      chute.update();
      telemetry.update();
      sleep(20);
    }
  }
}
