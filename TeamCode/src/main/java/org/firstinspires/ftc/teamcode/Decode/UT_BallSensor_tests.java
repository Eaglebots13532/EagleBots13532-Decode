// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Decode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class UT_BallSensor_tests extends LinearOpMode {

  DC_BallSensor Balls = new DC_BallSensor(this);

  @Override
  public void runOpMode() throws InterruptedException {

    Balls.SensorInit();
    waitForStart();
    while (opModeIsActive()) {
      double BC = Balls.Sample();
      telemetry.addData("color", Balls.Sample());
      if (BC < 5.0) telemetry.addLine("No Ball");
      else if (BC < 7.0) telemetry.addLine("Green");
      else if (BC < 8.0) telemetry.addLine("Purple)");
      telemetry.addData("distance", Balls.hopper);
      telemetry.addData("distance", Balls.Present());
      telemetry.update();
    }
  }
}
