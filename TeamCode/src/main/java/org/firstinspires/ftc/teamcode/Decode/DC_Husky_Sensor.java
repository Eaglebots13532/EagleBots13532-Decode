// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Decode;

// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/*
 * This Class uses the DFRobot HuskyLens.
 *
 * The HuskyLens is a Vision Sensor with a built-in object detection model.  It can
 * detect a number of predefined objects and AprilTags in the 36h11 family, Object Colors
 *
 * HuskyLens is configured for a name of "huskylens".
 * camera image size is 320 by 240
 */

public class DC_Husky_Sensor {

  private final int READ_PERIOD = 1;

  private HuskyLens huskyLens;
  private String algorithm;

  // Default constructor
  public DC_Husky_Sensor(String Algorithm, LinearOpMode opmode) {
    hskOp = opmode;
    this.algorithm = Algorithm;
  }

  /* Declare OpMode members.
   * gain access to methods in the calling OpMode.
   */
  private LinearOpMode hskOp = null;

  /*
   * you may set the rate Limit. Rate limits the reads solely to allow a user time to observe
   * what is happening on the Driver Station telemetry.
   * Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);
   *  Typical applications would not likely rate limit.
   */
  /*
   * color - object color recognition
   * april - april tag recognition
   */

  enum b {
    purple,
    green
  }

  private static final int IMG_CENTER_X = 160; // Center of the 320px screen
  private static final int TARGET_WIDTH = 100; // Desired pixel width (distance proxy)
  private static final double BEARING_GAIN = 0.01; // Proportional gain for turning speed
  private static final double RANGE_GAIN = 0.005; // Proportional gain for forward speed
  private static final double MIN_FORWARD_SPEED = 0.1; // Minimum speed to overcome friction

  private int objId = 0;
  private int height = 0;
  private int width = 0;
  private int left = 0;
  private int top = 0;
  private int x = 0;
  private int y = 0;
  private int IdSeen = 0;
  private int GTop = 0;
  private int GId = 0;

  public int side = 0; // 1 red 2 Blue

  public int green = 1;
  public int purple = 3;
  public int Red = 1;
  public int Blue = 2;
  public int Obel = 3; // Obelisk
  // this is the forward speed
  public double forwardSpeed = 0.0;
  // this is the turn speed
  public double turnSpeed = 0.0;

  public void initHuskyLens() {
    huskyLens = hskOp.hardwareMap.get(HuskyLens.class, "huskyLens");
    /*
     * The device uses the concept of an algorithm to determine what types of
     * objects it will look for and/or what mode it is in.
     * Other algorithm choices for FTC might be: OBJECT_RECOGNITION, COLOR_RECOGNITION or OBJECT_CLASSIFICATION.
     */
    if (algorithm == "april") huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
    if (algorithm == "color") huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
    if (algorithm == "track") huskyLens.selectAlgorithm(HuskyLens.Algorithm.OBJECT_TRACKING);
    /*
     * Immediately expire so that the first time through we'll do the read.
    // .expire();

    /*
     * Basic check to see if the device is alive and communicating.  This is not
     * technically necessary here as the HuskyLens class does this in its
     * doInitialization() method which is called when the device is pulled out of
     * the hardware map.  However, sometimes it's unclear why a device reports as
     * failing on initialization.  In the case of this device, it's because the
     * call to knock() failed.
     */
    if (!huskyLens.knock()) {
      hskOp.telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
    } else {
      hskOp.telemetry.addData(">>", "Husky Lens camera initiated");
    }

    hskOp.telemetry.update();
  }

  // get the blocks from camera and set variables from the largest seen object
  // setBlocks may not retrieve from a singular object as perceived by movement
  // towards object from multiple seen objects
  public void getBlocks() {

    /*
     * All algorithms, except for LINE_TRACKING, return a list of Blocks where aBlock represents the outline of a recognized object along with its ID number.
     * ID numbers allow you to identify what the device saw.  See the HuskyLens documentation
     *
     *  referenced in the header comment above for more information on IDs and how to
     * assign them to objects.
     *
     * Returns an empty array if no objects are seen.
     */
    HuskyLens.Block[] blocks = huskyLens.blocks();
    GTop = 0;
    GId = 0; // reset greater than
    IdSeen = blocks.length; // return the largest size
    // process for Tags and balls
    if (algorithm == "color") {
      for (int i = 0; i < blocks.length; i++) {
        if (blocks[i].id == green || blocks[i].id == purple) {
          if (blocks[i].top > GTop) { // larger = closest ball
            GTop = blocks[i].top;
            GId = blocks[i].id;
            objId = GId;
            height = blocks[i].height;
            width = blocks[i].width;
            left = blocks[i].left;
            top = blocks[i].top;
            x = blocks[i].x;
            y = blocks[i].y;
          } else {
            objId = 0;
          } // save largest
        } // OK purple or green set green and purple to object Id
      } // // return the largest object found
    } // algorithm color for ball objects
    if (algorithm == "april") {
      // Search for target tags
      for (HuskyLens.Block block : blocks) {
        // side is red in autonomous and read in teleOp
        // this gives the range and bearing to drive system
        if (side == 0) {
          if (block.id == 1 || block.id == Red || block.id == Blue || block.id == Obel) {
            side = block.id;
          }
        } else if (block.id == side) {
          x = block.x;
          width = block.width;
          FSTS(); // set speeds
          break; // Track the first one we find
        }
      }
    }
  } // end set blocks

  public int GId() {
    return GId;
  }

  public int getLeft() {
    return left;
  }

  public int getTop() {
    return top;
  }

  public int getX() {
    return x;
  }

  public int getY() {
    return y;
  }

  // movement calculations
  // Size position in image returned as area max 76800
  public int size() {
    return height * width;
  }

  // Top 0 bottom 240 noralized  0 - 1.0
  public double topPos() {
    return (double) top / 240.0;
  }

  // Left 0 right 320 return value -1 to 1 from center bottom
  public double leftPos() {
    return (double) left / 320.0;
  }

  public void FSTS() {
    // Calculate speeds using proportional control
    double bearingError = x - IMG_CENTER_X;
    turnSpeed = bearingError * BEARING_GAIN;

    double rangeError = TARGET_WIDTH - width;
    forwardSpeed = rangeError * RANGE_GAIN;

    // Add minimum speed threshold to ensure the robot moves
    if (Math.abs(forwardSpeed) < MIN_FORWARD_SPEED) {
      forwardSpeed = (forwardSpeed > 0) ? MIN_FORWARD_SPEED : -MIN_FORWARD_SPEED;
    }
  }

  private double map(double value) {
    /* simplified -1 + value * 2 from following:
    Output value = outputStart +
     ((value - inputStart) / (inputEnd - inputStart))
     * (outputEnd - outputStart) */
    return -1 + value * 2;
  }
} // end Husky decode class
