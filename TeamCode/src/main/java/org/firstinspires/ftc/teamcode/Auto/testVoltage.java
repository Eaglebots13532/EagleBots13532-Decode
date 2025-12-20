// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@Autonomous(name = "output position")
public class  testVoltage extends LinearOpMode {

  @Override
  public void runOpMode() throws InterruptedException {
    VoltageSensor voltageSensor = hardwareMap.get(VoltageSensor.class,"Control Hub");
    double currentVoltage = voltageSensor.getVoltage();

    waitForStart();
    while (opModeIsActive()) {
      telemetry.addData("current voltage: ",currentVoltage);
      telemetry.update();
    }// While opmode active
  }// run Op Mode
}// OutputXY end
