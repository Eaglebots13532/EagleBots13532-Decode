// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.drivers.odo;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class CarlOdometryExampleImplementation {

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
  WebCamCarlCoaxSwerve webcam;
  double lastRobotX = 0;
  double lastRobotY = 0;
  double lastRobotYaw = 0;
  GoBildaPinpointDriver ppo;

  public CarlOdometryExampleImplementation(WebCamCarlCoaxSwerve webcam) {
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

  public void resetFieldWebcamRedBin() {
    webcam.update();
    Double dist = webcam.getAprilDistance(24);
    Double bearing = webcam.getAprilBearing(24);

    if (dist != null && bearing != null) {
      double bearingRad = Math.toRadians(bearing);

      fieldX = aprilX - dist * Math.sin(bearingRad);
      fieldY = aprilY - dist * Math.cos(bearingRad);
      fieldYaw = bearing + aprilYaw;

      resetXValue = robotX;
      resetYValue = robotY;
      resetYawValue = robotYaw;
    }
  }

  public void update() {
    // Update odometry
    ppo.update();

    // Get current robot position
    robotX = ppo.getPosX(DistanceUnit.CM) - resetXValue;
    robotY = ppo.getPosY(DistanceUnit.CM) - resetYValue;
    robotYaw = ppo.getHeading(AngleUnit.DEGREES) - resetYawValue;

    // Compute deltas
    double dx = robotX - lastRobotX;
    double dy = robotY - lastRobotY;

    // Convert yaw to radians for heading calculations
    double headingRad = Math.toRadians(robotYaw);

    // Rotate robot-relative movement into field frame
    double fieldDeltaX = dx * Math.cos(headingRad) - dy * Math.sin(headingRad);
    double fieldDeltaY = dx * Math.sin(headingRad) + dy * Math.cos(headingRad);

    // Accumulate into field position
    fieldX += fieldDeltaX;
    fieldY += fieldDeltaY;
    fieldYaw = robotYaw;

    // Store last values for next loop
    lastRobotX = robotX;
    lastRobotY = robotY;
    lastRobotYaw = robotYaw;
  }

  public double getFieldX() {
    ;
    update();
    return fieldX;
  }

  public double getFieldY() {
    update();
    return fieldY;
  }

  public double getFieldYaw() {
    update();
    return fieldYaw;
  }

  public double getShootingDistance() {
    update();
    double aprilPosX;
    double aprilPosY;
    double aprilDistance;

    aprilPosX = fieldX - aprilX;
    aprilPosY = fieldY - aprilY;

    aprilDistance = Math.hypot(aprilPosX, aprilPosY);
    return aprilDistance;
  }

  public double getShootingX() {
    double aprilPosX;

    aprilPosX = fieldX - aprilX;
    return aprilPosX;
  }

  public double getShootingY() {
    double aprilPosY;

    aprilPosY = fieldY - aprilY;
    return aprilPosY;
  }
}
