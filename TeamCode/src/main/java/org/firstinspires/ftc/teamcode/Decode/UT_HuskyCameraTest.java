// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Decode; // Copyright (c) 2024-2025 FTC 13532

// All rights reserved.

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.concurrent.TimeUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

/*
 * This OpMode illustrates how to use the DFRobot HuskyLens.
 *
 * The HuskyLens is a Vision Sensor with a built-in object detection model.  It can
 * detect a number of predefined objects and AprilTags in the 36h11 family, can
 * recognize colors, and can be trained to detect custom objects. See this website for
 * documentation: https://wiki.dfrobot.com/HUSKYLENS_V1.0_SKU_SEN0305_SEN0336
 *
 * For detailed instructions on how a HuskyLens is used in FTC, please see this tutorial:
 * https://ftc-docs.firstinspires.org/en/latest/devices/huskylens/huskylens.html
 *
 * This sample illustrates how to detect AprilTags, but can be used to detect other types
 * of objects by changing the algorithm. It assumes that the HuskyLens is configured with
 * a name of "huskylens".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
@TeleOp(name = "Test: HuskyLens", group = "Sensor")
public class UT_HuskyCameraTest extends LinearOpMode {

  private final int READ_PERIOD = 1;

  private HuskyLens huskyLens;

  private static final int IMG_CENTER_X = 160; // Center of the 320px screen
  private static final int TARGET_WIDTH = 100; // Desired pixel width (distance proxy)
  private static final double BEARING_GAIN = 0.01; // Proportional gain for turning speed
  private static final double RANGE_GAIN = 0.005; // Proportional gain for forward speed
  private static final double MIN_FORWARD_SPEED = 0.1; // Minimum speed to overcome friction

  @Override
  public void runOpMode() {
    huskyLens = hardwareMap.get(HuskyLens.class, "huskyLens");

    /*
     * This sample rate limits the reads solely to allow a user time to observe
     * what is happening on the Driver Station telemetry.  Typical applications
     * would not likely rate limit.
     */
    Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);

    /*
     * Immediately expire so that the first time through we'll do the read.
     */
    rateLimit.expire();

    /*
     * Basic check to see if the device is alive and communicating.  This is not
     * technically necessary here as the HuskyLens class does this in its
     * doInitialization() method which is called when the device is pulled out of
     * the hardware map.  However, sometimes it's unclear why a device reports as
     * failing on initialization.  In the case of this device, it's because the
     * call to knock() failed.
     */
    if (!huskyLens.knock()) {
      telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
    } else {
      telemetry.addData(">>", "Press start to continue");
    }

    /*
     * The device uses the concept of an algorithm to determine what types of
     * objects it will look for and/or what mode it is in.  The algorithm may be
     * selected using the scroll wheel on the device, or via software as shown in
     * the call to selectAlgorithm().
     *
     * The SDK itself does not assume that the user wants a particular algorithm on
     * startup, and hence does not set an algorithm.
     *
     * Users, should, in general, explicitly choose the algorithm they want to use
     * within the OpMode by calling selectAlgorithm() and passing it one of the values
     * found in the enumeration HuskyLens.Algorithm.
     *
     * Other algorithm choices for FTC might be: OBJECT_RECOGNITION, COLOR_RECOGNITION or OBJECT_CLASSIFICATION.
     */
    // huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
    huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

    telemetry.update();
    waitForStart();

    /*
     * Looking for AprilTags per the call to selectAlgorithm() above.  A handy grid
     * for testing may be found at https://wiki.dfrobot.com/HUSKYLENS_V1.0_SKU_SEN0305_SEN0336#target_20.
     *
     * Note again that the device only recognizes the 36h11 family of tags out of the box.
     */
    while (opModeIsActive()) {
      if (!rateLimit.hasExpired()) {
        continue;
      }
      rateLimit.reset();

      /*
       * All algorithms, except for LINE_TRACKING, return a list of Blocks where a
       * Block represents the outline of a recognized object along with its ID number.
       * ID numbers allow you to identify what the device saw.  See the HuskyLens documentation
       * referenced in the header comment above for more information on IDs and how to
       * assign them to objects.
       *
       * Returns an empty array if no objects are seen.
       */
      int targetX = 0;
      int targetWidth = 0;
      boolean targetFound = false;
      HuskyLens.Block[] blocks = huskyLens.blocks();
      telemetry.addData("Block count", blocks.length);

      for (int i = 0; i < blocks.length; i++) {
        telemetry.addData("Block", blocks[i].toString());
        if (blocks[i].id == 1
            || blocks[i].id == 2
            || blocks[i].id == 3
            || blocks[i].id == 4
            || blocks[i].id == 5
            || blocks[i].id == 6) {
          targetX = blocks[i].x;
          targetWidth = blocks[i].width;
          targetFound = true;
          /*
           * Here inside the FOR loop, you could save or evaluate specific info for the currently recognized Bounding Box:
           * - blocks[i].width and blocks[i].height   (size of box, in pixels)
           * - blocks[i].left and blocks[i].top       (edges of box)
           * - blocks[i].x and blocks[i].y            (center location)
           * - blocks[i].id                           (Color ID)
           *
           * These values have Java type int (integer).
           */
          telemetry.addData("Color", blocks[i].id); //                       (Color ID)
          telemetry.addData("X", blocks[i].x);
          telemetry.addData("Y", blocks[i].y);
          telemetry.addData("h", blocks[i].height);
          telemetry.addData("w", blocks[i].width);
          telemetry.addData("T", blocks[i].top);
          telemetry.addData("L", blocks[i].left);
          break; // Track the first one we find
        }
      }

      telemetry.update();
      if (targetFound) {
        // Calculate speeds using proportional control
        double bearingError = targetX - IMG_CENTER_X;
        double turnSpeed = bearingError * BEARING_GAIN;

        double rangeError = TARGET_WIDTH - targetWidth;
        double forwardSpeed = rangeError * RANGE_GAIN;

        // Add minimum speed threshold to ensure the robot moves
        if (Math.abs(forwardSpeed) < MIN_FORWARD_SPEED) {
          forwardSpeed = (forwardSpeed > 0) ? MIN_FORWARD_SPEED : -MIN_FORWARD_SPEED;
        }

        // Use the Hardware class's drive method
        // robot.drive(forwardSpeed, turnSpeed);

        telemetry.addData("Target Found", "ID %d", blocks[0].id);
        telemetry.addData("Forward Speed", "%.2f", forwardSpeed);
        telemetry.addData("Turn Speed", "%.2f", turnSpeed);
      } else {
        // If no target is found, stop or execute a search pattern
        // robot.drive(0, 0);
        telemetry.addData("Status", "Searching...");
      }
      telemetry.update();
    }
  }
} // Husky Lens Test
