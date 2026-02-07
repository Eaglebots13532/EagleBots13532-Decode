// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Decode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Decode.wpilib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.Decode.wpilib.util.Units;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

/*
 * This OpMode illustrates how to use the GoBildaPinpoint
 *
 * The OpMode assumes that the sensor is configured with a name of "pinpoint".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 *
 * See the sensor's product page: https://www.gobilda.com/pinpoint-odometry-computer-imu-sensor-fusion-for-2-wheel-odometry/
 */
@TeleOp(name = "Show GoBilda Pinpoint", group = "Sensor")
public class UT_SensorGoBildaPinpoint extends OpMode {
  // Create an instance of the sensor
  GoBildaPinpointDriver pinpoint;

  @Override
  public void init() {
    // Get a reference to the sensor
    pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

    // Configure the sensor
    configurePinpoint();

    // Set the location of the robot - this should be the place you are starting the robot from
    pinpoint.setPose(Pose2d.kZero);
  }

  @Override
  public void loop() {
    telemetry.addLine("Push your robot around to see it track");
    telemetry.addLine("Press A to reset the position");
    if (gamepad1.a) {
      // You could use readings from April Tags here to give a new known position to the pinpoint
      pinpoint.setPose(Pose2d.kZero);
    }
    pinpoint.update();
    Pose2d pose2D = pinpoint.getPose();

    telemetry.addData("X coordinate (IN)", Units.metersToInches(pose2D.getX()));
    telemetry.addData("Y coordinate (IN)", Units.metersToInches(pose2D.getY()));
    telemetry.addData("Heading angle (DEGREES)", pose2D.getRotation().getDegrees());
  }

  public void configurePinpoint() {
    /*
     *  Set the odometry pod positions relative to the point that you want the position to be measured from.
     *
     *  The X pod offset refers to how far sideways from the tracking point the X (forward) odometry pod is.
     *  Left of the center is a positive number, right of center is a negative number.
     *
     *  The Y pod offset refers to how far forwards from the tracking point the Y (strafe) odometry pod is.
     *  Forward of center is a positive number, backwards is a negative number.
     */
    //        pinpoint.setOffsets(-2.0, 0.0, DistanceUnit.INCH); //these are tuned for
    // 3110-0002-0001 Product Insight #1

    /*
     * Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
     * the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
     * If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
     * number of ticks per unit of your odometry pod.  For example:
     *     pinpoint.setEncoderResolution(13.26291192, DistanceUnit.MM);
     */
    pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

    /*
     * Set the direction that each of the two odometry pods count. The X (forward) pod should
     * increase when you move the robot forward. And the Y (strafe) pod should increase when
     * you move the robot to the left.
     */
    pinpoint.setEncoderDirections(
        GoBildaPinpointDriver.EncoderDirection.FORWARD,
        GoBildaPinpointDriver.EncoderDirection.FORWARD);

    /*
     * Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
     * The IMU will automatically calibrate when first powered on, but recalibrating before running
     * the robot is a good idea to ensure that the calibration is "good".
     * resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
     * This is recommended before you run your autonomous, as a bad initial calibration can cause
     * an incorrect starting value for x, y, and heading.
     */
    pinpoint.resetPosAndIMU();
  }
}
