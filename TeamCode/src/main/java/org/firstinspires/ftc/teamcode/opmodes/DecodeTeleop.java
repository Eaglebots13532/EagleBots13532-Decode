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

      // Capture gamepad inputs by asserting appropriate flags for later processing
      sm.captureInputs();
      sm.processState();

      sleep(20); // 50Hz loop
    }
  }
}
