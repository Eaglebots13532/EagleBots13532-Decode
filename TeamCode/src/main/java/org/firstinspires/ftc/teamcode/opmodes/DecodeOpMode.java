// Copyright (c) 2025-2026 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.chute.Chute;
import org.firstinspires.ftc.teamcode.chute.ChuteController;
import org.firstinspires.ftc.teamcode.chute.FtcMotor;
import org.firstinspires.ftc.teamcode.chute.FtcPotentiometer;

@TeleOp(name = "Decode OpMode")
public class DecodeOpMode extends LinearOpMode {

  @Override
  public void runOpMode() {
    
    waitForStart();

    while (opModeIsActive()) {
      

      sleep(20); // 50Hz loop
    }
  }
}
