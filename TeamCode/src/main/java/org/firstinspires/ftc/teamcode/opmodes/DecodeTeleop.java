// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
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
    // --- Hardware ---
    CRServo chuteMotor = hardwareMap.get(CRServo.class, "chute");
    AnalogInput chutePot = hardwareMap.get(AnalogInput.class, "CP");
    FtcPotentiometer pot = new FtcPotentiometer(chutePot, POT_WRAP_AMOUNT);

    // --- Subsystems ---
    ChuteDriver chute = new ChuteDriver(chuteMotor, pot, telemetry);
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
          // Fired on A button press (rising edge)
          @Override
          public void onTogglePrimary(boolean active) {}

          // Fired on B button press (rising edge)
          @Override
          public void onToggleSecondary(boolean active) {}

          // Fired on X button press (rising edge) -- send chute home
          @Override
          public void onActionX() {
            if (!chuteInputsLocked) {
              chuteInputsLocked = true;
              chute.goHome();
            }
          }

          // Fired on Y button press (rising edge) -- emergency stop, always fires
          @Override
          public void onActionY() {
            chute.stop();
          }

          // Fired on dpad up press (rising edge) -- extend chute by CHUTE_STEP
          @Override
          public void onIncrementUp() {
            if (!chuteInputsLocked) {
              chuteInputsLocked = true;
              chute.goToPosition(chute.getPosition() + CHUTE_STEP);
            }
          }

          // Fired on dpad down press (rising edge) -- retract chute by CHUTE_STEP
          @Override
          public void onIncrementDown() {
            if (!chuteInputsLocked) {
              chuteInputsLocked = true;
              double target = chute.getPosition() - CHUTE_STEP;
              if (target < 0.001) {
                chute.goHome();
              } else {
                chute.goToPosition(target);
              }
            }
          }

          // Fired on dpad left press (rising edge)
          @Override
          public void onCycleLeft() {}

          // Fired on dpad right press (rising edge)
          @Override
          public void onCycleRight() {}

          // Fired while left trigger is held past deadzone
          @Override
          public void onModifierLeft(float value) {}

          // Fired while right trigger is held past deadzone
          @Override
          public void onModifierRight(float value) {}
        });

    telemetry.addLine("Initialized -- waiting for start");
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
      sm.captureInputs();
      sm.processState();
      chute.update();
      telemetry.update();
      sleep(20);
    }
  }
}
