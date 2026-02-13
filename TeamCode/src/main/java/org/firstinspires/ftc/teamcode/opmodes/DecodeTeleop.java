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

      sleep(20); // 50Hz loop
    }
  }
}
