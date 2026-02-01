// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.FireBall;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
	Motor Config
	0 - Lmotor white
	1 - RMotor black
	2 - FMotor red
	3 - BMotor blue
	4 - LiftMotor

	Servo Config
	0 - Claw

	Digital Config
	0 - Home

	I2C Config
	0 - 0- IMU
	0 - 1 - leftDist
	1 - 0 - backDist
	2 - 0 - rightDist
	3 - 0 - V3color
*/

@TeleOp
public class FireballTeleOp extends LinearOpMode {

  // Lets this program call functions inside of DriveConfig
  FireballConfig Eagle = new FireballConfig(this);

  @Override
  public void runOpMode() {
    Eagle.init(); // initialize the HARDWARE AND SENSORS

    // Wait for the game to start (driver presses PLAY and runs until STOP is pressed)
    waitForStart();

    while (opModeIsActive()) {

      Eagle.move(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.a);
    } // end while opModeIsActive
  } // end void runOpMode
} // Eagle drive  class
