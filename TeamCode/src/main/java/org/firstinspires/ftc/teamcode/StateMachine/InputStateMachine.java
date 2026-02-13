// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.StateMachine;

import com.qualcomm.robotcore.hardware.Gamepad;

public class InputStateMachine {
  private Gamepad gamepad1;
  private Gamepad gamepad2;

  public InputStateMachine(Gamepad gamepad1, Gamepad gamepad2) {
    this.gamepad1 = gamepad1;
    this.gamepad2 = gamepad2;
  }

  public void captureInputs() {
    if (gamepad1.a) {}

    if (gamepad1.b) {}

    if (gamepad1.dpad_down) {}

    if (gamepad1.dpad_left) {}

    if (gamepad1.dpad_right) {}

    if (gamepad1.dpad_up) {}

    if (gamepad1.left_trigger > 0.1) {}

    if (gamepad1.right_trigger > 0.1) {}
  }

  public void processState() {}
}
