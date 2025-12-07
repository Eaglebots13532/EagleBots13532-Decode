// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Decode;

// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/*
 * This OpMode shows how to use a color sensor in a generic
 * way, regardless of which particular make or model of color sensor is used. The OpMode
 * assumes that the color sensor is configured with a name of "sensor_color".
 *
 */

public class DC_BallSensor {

  // Default constructor
  public DC_BallSensor(LinearOpMode opmode) {
    myOp = opmode;
  }

  /* Declare OpMode members.
   * gain access to methods in the calling OpMode.
   */
  private LinearOpMode myOp = null;

  /** The colorSensor field will contain a reference to our color sensor hardware object */
  NormalizedColorSensor colorSensor;

  // DistanceSensor distance;
  // RevColorSensorV3 colorSensor;
  // Once per loop, we will update this hsvValues array. The first element (0) will contain the
  // hue, the second element (1) will contain the saturation, and the third element (2) will
  // contain the value.
  final float[] hsvValues = new float[3];
  private double BallColor = hsvValues[2];
  public double hopper = 0.0;

  public void SensorInit() {

    // Get a reference to our sensor object. It's recommended to use NormalizedColorSensor over
    // ColorSensor, because NormalizedColorSensor consistently gives values between 0 and 1, while
    // the values you get from ColorSensor are dependent on the specific sensor you're using.
    // colorSensor = myOp.hardwareMap.get(NormalizedColorSensor.class, "ballSensor");
    colorSensor = myOp.hardwareMap.get(NormalizedColorSensor.class, "ballSensor");

    // Turn on light
    if (colorSensor instanceof SwitchableLight) {
      ((SwitchableLight) colorSensor).enableLight(true);
    }
  }

  // since this is used mostly in a while it returns the not, if ball is present false
  public boolean Present() {
    Boolean go = true;
    hopper = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);
    if (hopper < 6.0) go = false;
    return go;
  }

  protected double Sample() {
    // Get the normalized colors from the sensor
    NormalizedRGBA colors = colorSensor.getNormalizedColors();
    // Update the hsvValues array by passing it to Color.colorToHSV()
    Color.colorToHSV(colors.toColor(), hsvValues);
    BallColor = colors.alpha * 100.0;
    return BallColor;
  }
} // End Ball_Sensor
