// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Decode; // Copyright (c) 2024-2025 FTC 13532

// All rights reserved.

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Decode.wpilib.geometry.Rotation2d;

public class DC_Odometry_Sensor {
  private LinearOpMode myOp = null;

  private double x = 0.0; // x pos
  private double y = 0.0; // y pos
  private Rotation2d b ; // heading
  private double lx = 0.0; // x pos
  private double ly = 0.0; // y pos
  private Rotation2d lb ; // heading

  // Define a constructor that allows the OpMode to pass a reference to itself.
  public DC_Odometry_Sensor(LinearOpMode opmode) {
    myOp = opmode;
  }

  // ---
  GoBildaPinpointDriver ppo; // Declare OpMode member for the Odometry Computer

  // ---
  // Methods used in Swerve  robot
  // -----------------------------
  public void DoInit() {
    // Initialize GoBilda PinPoint Pose computer
    ppo = myOp.hardwareMap.get(GoBildaPinpointDriver.class, "odo");
    // offsets in inch from center
    //    ppo.setOffsets(1, -0.0,DistanceUnit.INCH);
    // set pinpoint resolution
    // ppo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
    ppo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
    // Set the direction that each of the two odometry pods count
    ppo.setEncoderDirections(
        GoBildaPinpointDriver.EncoderDirection.FORWARD,
        GoBildaPinpointDriver.EncoderDirection.FORWARD);
    ppo.resetPosAndIMU();
    myOp.telemetry.addData("odometry status:", ppo.getDeviceStatus());
    myOp.telemetry.update();
  }

  public void resetPPO() {
    ppo.resetPosAndIMU();
    myOp.telemetry.addData("odometry status:", ppo.getDeviceStatus());
    myOp.telemetry.update();
  }

  public void positionXY() {
    // update position data
    //ppo.getPosition();
    ppo.update();
    lx = x;
    ly = y;
    lb = b; // save last values
    x = ppo.getPosX();
    y = ppo.getPosY();
    b = ppo.getHeading();

  }

  public double getx() {
    return x;
  }

  public double gety() {
    return y;
  }

  public double getBearing() {
    return b;
  }

  // Get positions for GoBilda Pinpoint current Position (x & y in cm, and heading in degrees)
  protected double getXPosition() {
    ppo.update();
    Pose2D pos = ppo.getPosition();
    return pos.getX(DistanceUnit.INCH);
  }

  protected double getYPosition() {
    ppo.update();
    Pose2D pos = ppo.getPosition();
    return pos.getY(DistanceUnit.INCH);
  }

  // returns heading in radians
  public double getHeading() {
    ppo.update();
    Pose2D pos = ppo.getPosition();
    return pos.getHeading(AngleUnit.DEGREES);
  }

  // return heading in  degrees
  public double getHeadDeg() {
    ppo.update();
    Pose2D pos = ppo.getPosition();
    return pos.getHeading(AngleUnit.DEGREES);
  }

  public double vector() {
    double xsqr = Math.pow(x - lx, 2.0);
    double ysqr = Math.pow(y - ly, 2.0);
    return Math.sqrt(xsqr + ysqr);
  }

  public double bearing() {
    return Math.atan2(y, x);
  }

  public double fieldCentricX(double gmX, double gmY) {
    // xx = xcosB - ysinB
    double botheading = getHeading();
    return gmX * Math.cos(-botheading) - gmY * Math.sin(-botheading);
  }

  // field centric driving
  public double fieldCentricY(double gmX, double gmY) {
    // yy = xsinB + ycosB
    double botheading = getHeading();
    return gmX * Math.sin(-botheading) + gmY * Math.cos(-botheading);
  }
} // end class Odometry Decode
