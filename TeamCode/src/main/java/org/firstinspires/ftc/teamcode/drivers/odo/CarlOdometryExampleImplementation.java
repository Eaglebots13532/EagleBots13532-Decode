// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.drivers.odo;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drivers.AprilDriver;

public class CarlOdometryExampleImplementation {
  /*
  double fieldX;
  double fieldY;
  double fieldYaw;
  double robotX;
  double robotY;
  double robotYaw;
  double aprilX = 141.33;
  double aprilY = 148.19;
  double aprilYaw = -45.0;

  // Used to reset robot XY and Yaw values when homing with camera by making them equal to the
  // current robot XY and Yaw values to offset
  double resetXValue = 0;
  double resetYValue = 0;
  double resetYawValue = 0;

  double lastRobotX = 0;
  double lastRobotY = 0;
  double lastRobotYaw = 0;
  */
  GoBildaPinpointDriver ppo;
  AprilDriver webcam;

  public CarlOdometryExampleImplementation(AprilDriver webcam) {
    this.webcam = webcam;
  }

  public void init(HardwareMap hwMap) {
    ppo = hwMap.get(GoBildaPinpointDriver.class, "odo");
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
  }

  double resetX;
  double resetY;
  double robotX;
  double robotY;

  public void updatePos() {
    robotY = ppo.getPosX(DistanceUnit.CM);
    robotX = -ppo.getPosY(DistanceUnit.CM);
  }

  public double getRobotPosX() {
    return robotX;
  }

  public double getRobotPosY() {
    return robotY;
  }

  public void resetFieldX() {
    resetX = robotX;
  }

  public void resetFieldY() {
    resetY = robotY;
  }

  public void resetFieldXY() {
    resetX = robotX;
    resetY = robotY;
  }

  double fieldX;
  double fieldY;

  public double getFieldX() {
    fieldX = getRobotPosX() - resetX;
    return fieldX;
  }

  public double getFieldY() {
    fieldY = getRobotPosY() - resetY;
    return fieldY;
  }

  public double getRobotYaw() {
    return -ppo.getHeading(AngleUnit.DEGREES);
  }

  double resetYaw;
  double cameraOffset;

  public void resetYaw() {
    resetYaw = getRobotYaw();
    cameraOffset = webcam.getBearingAprilTag();
  }

  public double getAdjustedYaw() {
    return -(getRobotYaw() - resetYaw + cameraOffset) + 45;
  }

  public double getResetYaw() {
    return resetYaw;
  }

  double finalFieldX;
  double finalFieldY;
  double cameraX;
  double cameraY;

  public void resetFieldPos(double x, double y) {
    resetFieldXY();
    webcam.getAprilTag();
    cameraX = x;
    cameraY = y;
  }

  public double getCameraX() {
    return cameraX;
  }

  public double getCameraY() {
    return cameraY;
  }

  public void trackFieldPos() {
    finalFieldX = (getFieldX() + cameraX);
    finalFieldY = (getFieldY() + cameraY);
  }

  public double getFinalFieldX() {
    return finalFieldX;
  }

  public double getFinalFieldY() {
    return finalFieldY;
  }

  double seekHeading;
  double goalX;
  double goalY;
  static final double blueAprilLocationX = 146; // Cm
  static final double blueAprilLocationY = 146; // Cm
  static final double blueAprilLocationYaw = 45; // Degrees

  public double getSeekHeading() {
    goalX = Math.abs(blueAprilLocationX - finalFieldX);
    goalY = Math.abs(blueAprilLocationY - finalFieldY);
    seekHeading = Math.atan(goalX / goalY);
    return seekHeading;
  }
}
