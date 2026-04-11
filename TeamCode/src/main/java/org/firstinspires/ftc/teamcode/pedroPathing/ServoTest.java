// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class ServoTest extends LinearOpMode {
  CRServo LFS = null;
  CRServo RFS = null;

  AnalogInput LFP = null;
  AnalogInput RFP = null;

  ElapsedTime Timer = new ElapsedTime();

  @Override
  public void runOpMode() throws InterruptedException {
    LFS = hardwareMap.get(CRServo.class, "LFS");
    RFS = hardwareMap.get(CRServo.class, "RFS");
    LFP = hardwareMap.get(AnalogInput.class, "LFP");
    RFP = hardwareMap.get(AnalogInput.class, "RFP");
    double maxLP = 0.0;
    double maxRP = 0.0;
    double minLP = 1.0;
    double minRP = 1.0;
    double cMxLp = 0.0;
    double readL = 0.0;
    double readR = 0.0;
    double active = 0.0;
    boolean onoff = false;
    waitForStart();
    LFS.setPower(.1);
    RFS.setPower(.1);
    Timer.reset();
    do {
      if (gamepad1.aWasPressed()) {
        if (onoff) onoff = false;
        else onoff = true;
      }
      if (onoff) {
        readL = LFP.getVoltage();
        if (readL > maxLP) maxLP = readL;
        if (readL < minLP) minLP = readL;
        readR = RFP.getVoltage();
        if (readR > maxRP) maxRP = readR;
        if (readR < minRP) minRP = readR;
        telemetry.addLine(". LFP: " + readL);
        telemetry.addLine("maxLP: " + maxLP);
        telemetry.addLine("minLP: " + minLP);
        telemetry.addLine("-------------------------");
        telemetry.addLine(". RFP: " + readR);
        telemetry.addLine("maxRP: " + maxRP);
        telemetry.addLine("minRP: " + minRP);
        telemetry.addLine("-------------------------");
        telemetry.addLine("Cycle:" + Timer.milliseconds());
        telemetry.addLine();
        telemetry.update();
        Timer.reset();
      }
      if (!onoff) {
        maxLP = 0.0;
        maxRP = 0.0;
        minLP = 1.0;
        minRP = 1.0;
      }
    } while (opModeIsActive());
  }
}
