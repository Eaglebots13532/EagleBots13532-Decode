// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Decode; // Copyright (c) 2024-2025 FTC 13532

// All rights reserved.

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class UT_Intake_Arm_Chute_Spin_Test extends LinearOpMode {
  private final DC_Intake_Launch game = new DC_Intake_Launch(this);

  @Override
  public void runOpMode() {
    // FtcDashboard dashboard = FtcDashboard.getInstance();
    // telemetry = dashboard.getTelemetry();

    game.InitIL();
    int spinVeloc = 0;
    double chuteDrv = 0.0;
    int armEnc = 0;
    boolean leftBump = false;
    boolean rightBump = false;
    int armPos = 0;

    waitForStart();
    // arm positioning
    while (opModeIsActive()) {
      telemetry.addLine(". . . Intake . . .");
      telemetry.addLine("Right Bumper 2 start 3 sec stop");
      if (gamepad2.rightBumperWasPressed()) {
        game.Intake();
        sleep(3000);
        game.IntakeStop();
      }
      telemetry.addLine(". . . gate . . . .");
      telemetry.addLine(" Right Button X 2 open close gate");
      if (gamepad2.xWasPressed()) {
        game.openGate();
        sleep(3000);
        game.closeGate();
      }
      telemetry.addLine(". . . Arm . . . .");
      telemetry.addLine("Left Stick 2 Y");
      double armDrv = -gamepad2.left_stick_y / 1.5;
      game.arm.setPower(armDrv);
      telemetry.addLine(". . . . . . . . . .");
      // chute positioning

      telemetry.addLine(". . hood . . .");
      telemetry.addLine("DPad +left -right 2");
      if (gamepad2.dpad_right) chuteDrv = .2;
      if (gamepad2.dpad_left) chuteDrv = -.2;
      game.chute.setPower(chuteDrv);
      sleep(100);
      chuteDrv = 0.0;
      game.chute.setPower(chuteDrv);
      telemetry.addLine(". . . Fly Wheel . . .");
      // spin velocity
      telemetry.addLine("DPad +UP -down 2");
      // int spinVeloc = (int) (gamepad2.left_trigger * 6000);
      if (gamepad2.dpad_up) spinVeloc += 100;
      if (gamepad2.dpad_down) spinVeloc -= 100;
      sleep(700);
      if (spinVeloc < 1400) spinVeloc = 1350;
      if (spinVeloc > 2100) spinVeloc = 2100;
      game.launch.setVelocity(spinVeloc);
      telemetry.addLine(". . . lift . . . .");
      telemetry.addLine("right trigger lift");
      int lift = (int) (-gamepad1.left_stick_y);
      game.tilt.setPosition(lift);
      telemetry.addData("request Velocity:", spinVeloc);
      telemetry.addData("Fly Wheel Velocity:", game.launch.getVelocity());
      telemetry.addData("Hood potentiometer:", game.chuteVal.getVoltage());
      telemetry.addData("Arm Encoder Position:", game.arm.getCurrentPosition());
      telemetry.addData("Arm Power:", armDrv);
      telemetry.addData("lift", lift);
      telemetry.update();
    }
  } // run OpMode
}
