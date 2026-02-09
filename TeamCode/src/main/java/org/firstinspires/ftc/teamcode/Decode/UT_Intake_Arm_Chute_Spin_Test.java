// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Decode; // Copyright (c) 2024-2025 FTC 13532

// All rights reserved.

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class UT_Intake_Arm_Chute_Spin_Test extends LinearOpMode {
  private final DC_Intake_Launch game = new DC_Intake_Launch(this);
  private final DC_chuteControl chute = new DC_chuteControl(this);

  @Override
  public void runOpMode() {
    // FtcDashboard dashboard = FtcDashboard.getInstance();
    // telemetry = dashboard.getTelemetry();

    game.InitIL();
    chute.initHood();
    int spinVeloc = 2000;
    double chuteDrv = 0.0;
    int armEnc = 0;
    boolean leftBump = false;
    boolean rightBump = false;
    int armPos = 0;
    boolean turnOffDrv = true;

    waitForStart();
    chute.HoodHome();
    sleep(500);
    // arm positioning
    while (opModeIsActive()) {
      telemetry.addLine(". . . Intake . . .");
      telemetry.addLine("Right Bumper 2 start 3 sec stop");
      if (gamepad2.right_bumper) {
        game.Intake();
        sleep(3000);
        game.IntakeStop();
      }
      telemetry.addLine(". . . Arm . . . .");
      telemetry.addLine("Left Stick 2 Y");
      double armDrv = -gamepad2.left_stick_y / 1.5;
      game.arm.setPower(armDrv);
      telemetry.addLine(". . . Hood . . . .");
      // Hood positioning
      telemetry.addLine("DPad +left -right 2");
      if (gamepad2.dpad_right) chuteDrv += 10.0;
      if (gamepad2.dpad_left) chuteDrv += -10.0;
      telemetry.addData("Hood Position request:", chuteDrv);
      if (chuteDrv > 0.0) chute.HoodPosition(chuteDrv);
      sleep(100);
      telemetry.addLine(". . . Fly Velocity . . .");
      // spin velocity
      telemetry.addLine("DPad +UP -down 2");
      // int spinVeloc = (int) (gamepad2.left_trigger * 6000);
      if (gamepad2.dpad_up) spinVeloc += 100;
      if (gamepad2.dpad_down) spinVeloc -= 100;
      sleep(700);
      if (spinVeloc < 1400) spinVeloc = 1400;
      if (spinVeloc > 2100) spinVeloc = 2100;
      if (turnOffDrv) game.launch.setVelocity(spinVeloc);
      telemetry.addLine(". . . . . . . . . .");
      telemetry.addLine("right trigger lift");
      int lift = (int) (-gamepad1.left_stick_y);
      game.tilt.setPosition(lift);
      telemetry.addData("request Velocity:", spinVeloc);
      telemetry.addData("Fly Wheel Velocity:", game.launch.getVelocity());
      telemetry.addData("Hood potentiometer:", chute.lastVolt);
      telemetry.addData("Arm Encoder Position:", game.arm.getCurrentPosition());
      telemetry.addData("Arm Power:", armDrv);
      telemetry.addData("lift", lift);
      telemetry.update();
    }
  } // run OpMode
}
