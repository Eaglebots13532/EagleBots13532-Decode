// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.HuskyLensOnApril;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Tank Robot HuskyLens Auton", group = "Robot")
public class TankAutonHusky extends LinearOpMode {
  HardwareTankBot robot = new HardwareTankBot();

  private static final int IMG_CENTER_X = 160; // Center of the 320px screen
  private static final int TARGET_WIDTH = 100; // Desired pixel width (distance proxy)
  private static final double BEARING_GAIN = 0.01; // Proportional gain for turning speed
  private static final double RANGE_GAIN = 0.005; // Proportional gain for forward speed
  private static final double MIN_FORWARD_SPEED = 0.1; // Minimum speed to overcome friction

  @Override
  public void runOpMode() {
    // Initialize the hardware
    robot.init(hardwareMap);
    telemetry.addData("Status", "Hardware Initialized. Waiting for start.");
    telemetry.update();

    waitForStart();

    while (opModeIsActive()) {
      HuskyLens.Block[] blocks = robot.huskylens.blocks();
      int targetX = 0;
      int targetWidth = 0;
      boolean targetFound = false;

      // Search for target tags
      for (HuskyLens.Block block : blocks) {
        if (block.id == 22 || block.id == 23 || block.id == 24) {
          targetX = block.x;
          targetWidth = block.width;
          targetFound = true;
          break; // Track the first one we find
        }
      }

      if (targetFound) {
        // Calculate speeds using proportional control
        double bearingError = targetX - IMG_CENTER_X;
        double turnSpeed = bearingError * BEARING_GAIN;

        double rangeError = TARGET_WIDTH - targetWidth;
        double forwardSpeed = rangeError * RANGE_GAIN;

        // Add minimum speed threshold to ensure the robot moves
        if (Math.abs(forwardSpeed) > 0.01) {
          forwardSpeed =
              (forwardSpeed > 0)
                  ? Math.max(MIN_FORWARD_SPEED, forwardSpeed)
                  : Math.min(-MIN_FORWARD_SPEED, forwardSpeed);
        }

        // Use the Hardware class's drive method
        robot.drive(forwardSpeed, turnSpeed);

        telemetry.addData("Target Found", "ID %d", blocks[0].id);
        telemetry.addData("Forward Speed", "%.2f", forwardSpeed);
        telemetry.addData("Turn Speed", "%.2f", turnSpeed);
      } else {
        // If no target is found, stop or execute a search pattern
        robot.drive(0, 0);
        telemetry.addData("Status", "Searching...");
      }
      telemetry.update();
    }
  }
}
