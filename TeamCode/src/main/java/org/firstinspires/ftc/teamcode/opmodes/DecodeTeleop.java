// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.StateMachine.InputStateMachine;

@TeleOp(name = "Decode Teleop")
public class DecodeTeleop extends LinearOpMode {

  @Override
  public void runOpMode() {
    InputStateMachine sm = new InputStateMachine(gamepad1, gamepad2);

    waitForStart();
    while (opModeIsActive()) {

      sm.setListener(
          new InputStateMachine.StateListener() {
            // Fired on A button press (rising edge)
            @Override
            public void onTogglePrimary(boolean active) {}

            // Fired on B button press (rising edge)
            @Override
            public void onToggleSecondary(boolean active) {}

            // Fired on dpad up press (rising edge)
            @Override
            public void onIncrementUp() {}

            // Fired on dpad down press (rising edge)
            @Override
            public void onIncrementDown() {}

            // Fired on dpad left press (rising edge)
            @Override
            public void onCycleLeft() {}

            // Fired on dpad right press (rising edge)
            @Override
            public void onCycleRight() {}

            // Fired while left trigger is held past deadzone - value is raw trigger position
            // (0.0–1.0)
            @Override
            public void onModifierLeft(float value) {}

            // Fired while right trigger is held past deadzone - value is raw trigger position
            // (0.0–1.0)
            @Override
            public void onModifierRight(float value) {}
          });

      sleep(20); // 50Hz loop
    }
  }
}
