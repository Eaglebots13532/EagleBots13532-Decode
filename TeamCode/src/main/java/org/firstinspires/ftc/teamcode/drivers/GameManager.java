// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.drivers;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drivers.wpilib.interpolation.InterpolatingDoubleTreeMap;

public class GameManager {
  private final AprilDriver april;
  private final GameDriver decode;

  private final Telemetry telemetry;

  public GameManager(AprilDriver april, GameDriver decode, Telemetry telemetry) {
    this.april = april;
    this.decode = decode;
    this.telemetry = telemetry;
  }

  /*

   read april tag red or blue range and angle offset.
   fly wheel velocity set by formula passing range value
   chute  angle set set by formula passing range value

   if position in field grid is known and robot outside
   april tag range set velocity and angle from field position

  */
  public void initTags() {
    april.initAprilTag();
  }

  public void getTagData() {

    april.getAprilTag();
  }

  public double range() {
    return april.getRange();
  }

  public int getTag() {
    return april.getMetaId();
  }

  public double setFlyWheel() {

    InterpolatingDoubleTreeMap distanceToFlywheelVelocity = new InterpolatingDoubleTreeMap();
    distanceToFlywheelVelocity.put(35.0, 1450.0);
    distanceToFlywheelVelocity.put(79.0, 1600.0);
    distanceToFlywheelVelocity.put(126.0, 1700.0);
    distanceToFlywheelVelocity.put(130.0, 1800.0);
    distanceToFlywheelVelocity.put(160.0, 1900.0);
    distanceToFlywheelVelocity.put(273.0, 2100.0);

    double flyvelocity = 1850;
    getTagData();
    // in teleOp we are facing the correct april tag
    // at present there is no check for match tag
    if (getTag() == 24 || getTag() == 20) {
      flyvelocity = 1.6374 * range() + 1450;
      // flyvelocity = distanceToFlywheelVelocity.get(range());
    }
    return flyvelocity;
  }

  public void setGPHood() {
    double gprange = decode.gp2LYjoy();
    if (gprange >= 0) {
      // Maps [0 to 1] -> [20 to 110]
      gprange = (gprange * 90.0) + 20.0;
      decode.HoodPosition((int) gprange);
    }
  }

  public void setCamHood() {
    getTagData();
    // in teleOp we are facing the correct april tag
    // at present there is no check for match tag
    if (getTag() == 24 || getTag() == 20) {
      // hoodRange = -.024 * range() + 2.7586;
      decode.HoodPosition((int) range());
    }
  }
} // end game manager
