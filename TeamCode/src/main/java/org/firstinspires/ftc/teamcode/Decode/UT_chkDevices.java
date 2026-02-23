// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Decode; // Copyright (c) 2024-2025 FTC 13532

// All rights reserved.

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@TeleOp
public class UT_chkDevices extends LinearOpMode {
  private final DC_Intake_Launch game = new DC_Intake_Launch(this);
  public VoltageSensor voltageSensor;

  @Override
  public void runOpMode() {
    // FtcDashboard dashboard = FtcDashboard.getInstance();
    // telemetry = dashboard.getTelemetry();

    game.InitIL();
    voltageSensor = hardwareMap.voltageSensor.iterator().next();
    int spinVeloc = 2000;
    double chuteDrv = 0.0;
    int armEnc = 0;
    boolean leftBump = false;
    boolean rightBump = false;
    int armPos = 0;
    boolean turnOnFly = false;
    boolean turnOnHood = false;

    waitForStart();
    // arm positioning
    while (opModeIsActive()) {
      if (gamepad2.xWasPressed()) turnOnFly = !turnOnFly;
      telemetry.addData("Fly motor", turnOnFly);
      if (gamepad2.yWasPressed()) turnOnHood = !turnOnHood;
      telemetry.addData("Hood Servo", turnOnHood);
      telemetry.addLine(". . . . Intake . . . .");
      telemetry.addLine("Right Bumper 2 start 3 sec stop");
      if (gamepad2.right_bumper) {
        game.Intake();
        sleep(3000);
        game.IntakeStop();
      }
      telemetry.addLine(". . . . gate. . . .");
      telemetry.addLine("b button 2 open 1 sec close 1 sec");
      if (gamepad2.bWasPressed()) {
        game.openGate();
        sleep(1000);
        game.closeGate();
        sleep(1000);
      }
      telemetry.addLine(". . . . Arm . . . .");
      telemetry.addLine("Left Stick 2 Y");
      double armDrv = -gamepad2.left_stick_y / 2.0;
      game.arm.setPower(armDrv);
      telemetry.addLine(". . . . Hood . . . .");
      // chute positioning
      telemetry.addLine("DPad +left -right 2");
      if (gamepad2.dpad_right) {
        chuteDrv = .2;
        if (turnOnHood) game.chuteMotor.setPower(chuteDrv);
      }
      if (gamepad2.dpad_left) {
        chuteDrv = -.2;
        if (turnOnHood) game.chuteMotor.setPower(chuteDrv);
      }
      sleep(100);
      game.chuteMotor.setPower(0.0);
      telemetry.addLine(". . . Fly Wheel . . . .");
      telemetry.addLine("DPad +UP -down 2");
      if (gamepad2.dpad_up) spinVeloc += 100;
      if (gamepad2.dpad_down) spinVeloc -= 100;
      if (spinVeloc < 1400) spinVeloc = 1400;
      if (spinVeloc > 2100) spinVeloc = 2100;
      if (turnOnFly) game.spinUp(spinVeloc);
      if (!turnOnFly) game.spinOff();
      telemetry.addLine(". . . . Tilt . . . .");
      telemetry.addLine("right stick y tilt");
      double lift = (-gamepad2.right_stick_y + 1.0) / 2.0;
      game.tilt.setPosition(lift);
      double flyvel = game.getFlyVel();
      sleep(100);
      telemetry.addData("Fly Request Velocity:", spinVeloc);
      telemetry.addData("Fly Present Velocity:", flyvel);
      double Hpot = game.chutePot.getVoltage();
      telemetry.addData("Hood potentiometer:", Hpot);
      double ArmEnc = game.getArmEnc();
      sleep(100);
      telemetry.addData("Arm Encoder Position:", ArmEnc);
      telemetry.addData("Arm Power:", armDrv);
      telemetry.addData("lift", lift);
      telemetry.addData("battery", voltageSensor.getVoltage());
      telemetry.update();
    }
  } // run OpMode
}
