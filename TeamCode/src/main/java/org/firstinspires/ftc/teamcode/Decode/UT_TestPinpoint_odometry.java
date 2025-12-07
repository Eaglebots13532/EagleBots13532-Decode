// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Decode; // Copyright (c) 2024-2025 FTC 13532

// All rights reserved.

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
Utility Test for Gobilda Pinpoint Odometry
*/
@TeleOp(name = "Test Odometry", group = "Utility Test")
public class UT_TestPinpoint_odometry extends LinearOpMode {

  /** The variable to store our instance of the odometry using PinPoint processor. */
  private DC_Odometry_Sensor odometry;

  /** The variable to store our instance of the vision portal. */
  @Override
  public void runOpMode() {

    odometry.DoInit();

    waitForStart();
    if (opModeIsActive()) {
      while (opModeIsActive()) {
        odometry.positionXY(); // read odometry data
        // output data
        double posX = odometry.getx();
        double posY = odometry.gety();
        double vect = odometry.vector();
        double bear = odometry.bearing();
        // Wait for the DS start button to be touched.
        telemetry.addLine("Odometry Data");
        telemetry.addData("X", posX);
        telemetry.addData("Y", posY);
        telemetry.addData("0 Bearing", bear);
        telemetry.addData("0 vector", vect);
        telemetry.addData("Heading", odometry.bearing());
        telemetry.addData("Vector", odometry.vector());
        telemetry.update();
        // Share the CPU.
        sleep(20);
      }
    }
  } // end method runOpMode()
} // end class
