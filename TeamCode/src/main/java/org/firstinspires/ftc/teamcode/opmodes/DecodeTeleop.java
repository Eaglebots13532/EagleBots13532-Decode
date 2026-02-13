// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Decode Teleop")
public class DecodeTeleop extends LinearOpMode {

  @Override
  public void runOpMode() {

    waitForStart();

    while (opModeIsActive()) {

      // Handle gamepad input
      handleControls()

      sleep(20); // 50Hz loop
    }
  }

  private void handleControls() {
    if (gamepad1.a) {
      
    }

    if (gamepad1.b) {
      
    }

    if (gamepad1.dpad_down) {
      
    }
  
    if (gamepad1.dpad_left) {
      
    }
  
    if (gamepad1.dpad_right) {
      
    }
  
    if (gamepad1.dpad_up) {

    }

    if (gamepad1.left_trigger > 0.1) {
      
    }
  
    if (gamepad1.right_trigger > 0.1) {

    }
  }
}
