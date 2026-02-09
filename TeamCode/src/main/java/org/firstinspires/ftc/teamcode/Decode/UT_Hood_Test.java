// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Decode; // Copyright (c) 2024-2025 FTC 13532

// All rights reserved.

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class UT_Hood_Test extends LinearOpMode {
  private final DC_Intake_Launch game = new DC_Intake_Launch(this);
  private final DC_chuteControl chute = new DC_chuteControl(this);

  @Override
  public void runOpMode() {
    // FtcDashboard dashboard = FtcDashboard.getInstance();
    // telemetry = dashboard.getTelemetry();

    game.InitIL();
    chute.initHood();

    double chuteDrv = 0.0;
    boolean turnOffDrv = true;

    waitForStart();
    chute.HoodHome();
    sleep(500);
    // arm positioning
    while (opModeIsActive()) {
      telemetry.addLine(". . . Hood . . . .");
      // Hood positioning
      telemetry.addLine("DPad +left -right 2");
      if (gamepad2.dpadRightWasPressed()) chuteDrv += 10.0;
      if (gamepad2.dpadLeftWasPressed()) chuteDrv += -10.0;
      chute.HoodPosition(chuteDrv);
      sleep(1000);
      if (!chute.goToTargetPos) chute.hoodStop();
      telemetry.addData("Hood Position request:", chuteDrv);
      telemetry.addData("Hood Position stop:", chute.goToTargetPos);
      telemetry.addData("Hood potentiometer:", chute.lastVolt);
      telemetry.update();
    }
  } // run OpMode
}
