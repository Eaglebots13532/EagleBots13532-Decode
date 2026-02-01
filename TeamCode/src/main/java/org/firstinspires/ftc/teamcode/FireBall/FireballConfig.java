// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.FireBall;

import static com.qualcomm.hardware.bosch.BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
import static com.qualcomm.hardware.bosch.BNO055IMU.AngleUnit.DEGREES;
import static com.qualcomm.hardware.bosch.BNO055IMU.SensorMode.IMU;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

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

*/

public class FireballConfig {

  /* Declare OpMode members. */
  private final LinearOpMode myOpMode; // gain access to methods in the calling OpMode.

  // Creates variables for the drive motors, set so nothing
  DcMotor FLMotor = null;
  DcMotor BRMotor = null;
  DcMotor BLMotor = null;
  DcMotor FRMotor = null;

  // IMU setups
  BNO055IMU imu;

  public double FLScaled, BLScaled, BRScaled, FRScaled;
  public double slowdown = 1.0;

  ElapsedTime runtime = new ElapsedTime();

  // Define a constructor that allows the OpMode to pass a reference to itself.
  public FireballConfig(LinearOpMode opmode) {
    myOpMode = opmode;
  }

  public void init() {
    // Define and Initialize Motors (note: need to use reference to actual OpMode).
    FLMotor = myOpMode.hardwareMap.get(DcMotor.class, "LMotor");
    BLMotor = myOpMode.hardwareMap.get(DcMotor.class, "BMotor");
    BRMotor = myOpMode.hardwareMap.get(DcMotor.class, "RMotor");
    FRMotor = myOpMode.hardwareMap.get(DcMotor.class, "FMotor");

    // Resets encoder
    FLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    BLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    BRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    FRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    // Set motor direction to move forward
    FLMotor.setDirection(DcMotor.Direction.FORWARD);
    BLMotor.setDirection(DcMotor.Direction.FORWARD);
    BRMotor.setDirection(DcMotor.Direction.REVERSE);
    FRMotor.setDirection(DcMotor.Direction.REVERSE);

    // Sets the mode of the motors
    FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    BLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    BRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    // Sets the parameters of the IMU in the control hub
    imu = myOpMode.hardwareMap.get(BNO055IMU.class, "imu");
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    parameters.mode = IMU;
    parameters.angleUnit = DEGREES;
    parameters.accelUnit = METERS_PERSEC_PERSEC;
    parameters.loggingEnabled = false;
    imu.initialize(parameters);

    myOpMode.telemetry.addData(">", "Hardware Initialized");
    myOpMode.telemetry.update();
  } // End init function

  // Sends info to datapad for debug and testing
  public void checkData() {
    myOpMode.telemetry.addLine();
    myOpMode.telemetry.addData("R>", FLMotor.getCurrentPosition());
    myOpMode.telemetry.addData("B>", BLMotor.getCurrentPosition());
    myOpMode.telemetry.addData("L>", BRMotor.getCurrentPosition());
    myOpMode.telemetry.addData("F>", FRMotor.getCurrentPosition());

    myOpMode.telemetry.addLine();
    myOpMode.telemetry.addData("IMU:", getHeading());
    myOpMode.telemetry.update();
  } // End checkData function

  public void move(double drive, double strafe, double turn, boolean boost) {
    double max;
    double leftAdj = 1;
    double rightAdj = 0.95;

    double FLRaw;
    double BLRaw;
    double BRRaw;
    double FRRaw;

    double FLScaled;
    double BLScaled;
    double BRScaled;
    double FRScaled;

    // when boost is true speeds to full speed otherwise reduced
    if (boost) {
      max = 0.75;
    } else {
      max = 0.33;
    }

    FLRaw = (drive - strafe - turn) * leftAdj;
    BLRaw = (drive + strafe - turn) * leftAdj;
    BRRaw = (drive - strafe + turn) * rightAdj;
    FRRaw = (drive + strafe + turn) * rightAdj;

    double rawMax =
        Math.max(
            Math.max(Math.abs(FLRaw), Math.abs(BLRaw)), Math.max(Math.abs(BRRaw), Math.abs(FRRaw)));

    if (rawMax > 1) {
      FLScaled = FLRaw / rawMax;
      BLScaled = BLRaw / rawMax;
      BRScaled = BRRaw / rawMax;
      FRScaled = FRRaw / rawMax;
    } else {
      FLScaled = FLRaw;
      BLScaled = BLRaw;
      BRScaled = BRRaw;
      FRScaled = FRRaw;
    }

    // send calculated power to wheels
    FLMotor.setPower(FLScaled * max);
    BLMotor.setPower(BLScaled * max);
    BRMotor.setPower(BRScaled * max);
    FRMotor.setPower(FRScaled * max);
  } // End move

  public void moveLeft(double drive, double strafe, double turnTarget) {
    while (myOpMode.opModeIsActive() && getHeading() < turnTarget) {
      double turnPower = getHeading() + turnTarget;

      move(drive, strafe, turnPower / 10, false);
    }
  }

  public void moveRight(double drive, double strafe, double turnTarget) {
    while (myOpMode.opModeIsActive() && getHeading() > turnTarget) {
      double turnPower = getHeading() - turnTarget;

      move(drive, strafe, turnPower / 50, false);
    }
  }

  // Stops all motors if necessary
  public void stopDrive() {
    FLMotor.setPower(0.0);
    BLMotor.setPower(0.0);
    BRMotor.setPower(0.0);
    FRMotor.setPower(0.0);
  } // End stopDrive

  // Return 1 = Red, 2 = Green, 3 = Blue

  public double getHeading() {
    Orientation angles =
        imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    double heading = angles.firstAngle;
    if (heading < -180) {
      heading += 360;
    } else if (heading > 180) {
      heading -= 360;
    }
    return heading;
  } // End getHeading

  public void fieldMove(
      double stick1Y, double stick1X, double stick2X, boolean boost, double imuHeading) {
    double headingRadians = 0;
    double headingPower = 0;
    double max = 0;
    double lrOfset = 0;
    double fbOfset = 0;

    // Change degrees to radians
    imuHeading *= (Math.PI / 180);

    if (stick1X == 0 && stick1Y == 0) {
      headingRadians = 0;
      headingPower = 0;
    } else {
      headingRadians = Math.atan2(stick1X, stick1Y);
      headingPower = Math.min(Math.sqrt((stick1X * stick1X) + (stick1Y * stick1Y)), 1);
    }

    // Calculate the power to individual motors
    double FLRaw =
        headingPower * -Math.sin(headingRadians + imuHeading + (Math.PI / 4)) - stick2X * 0.5;
    double BLRaw =
        headingPower * -Math.cos(headingRadians + imuHeading + (Math.PI / 4)) - stick2X * 0.5;
    double BRRaw =
        headingPower * Math.sin(headingRadians + imuHeading + (Math.PI / 4)) - stick2X * 0.5;
    double FRRaw =
        headingPower * Math.cos(headingRadians + imuHeading + (Math.PI / 4)) - stick2X * 0.5;

    double rawMax =
        Math.max(
            Math.max(Math.abs(FLRaw), Math.abs(BLRaw)), Math.max(Math.abs(BRRaw), Math.abs(FRRaw)));

    if (rawMax > 1) {
      FLScaled = FLRaw / rawMax;
      BLScaled = BLRaw / rawMax;
      BRScaled = BRRaw / rawMax;
      FRScaled = FRRaw / rawMax;
    } else {
      FLScaled = FLRaw;
      BLScaled = BLRaw;
      BRScaled = BRRaw;
      FRScaled = FRRaw;
    }

    if (boost) {
      max = 1;
    } else {
      max = 0.33;
    }

    if (myOpMode.gamepad1.dpad_right) {
      lrOfset = 0.25;
    } else if (myOpMode.gamepad1.dpad_left) {
      lrOfset = -0.25;
    } else {
      lrOfset = 0;
    }
    if (myOpMode.gamepad1.dpad_up) {
      fbOfset = 0.25;
    } else if (myOpMode.gamepad1.dpad_down) {
      fbOfset = -0.25;
    } else {
      fbOfset = 0;
    }

    FLMotor.setPower(FLScaled * max);
    BLMotor.setPower(BLScaled * max);
    BRMotor.setPower(BRScaled * max);
    FRMotor.setPower(FRScaled * max);
    /*
    FLMotor.setPower(0.1);
    BLMotor.setPower(0.1);
    BRMotor.setPower(0.1);
    FRMotor.setPower(0.1);
    */

  }
} // End class Fireball config
