// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.drivers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

/*
 * This OpMode illustrates the basics of AprilTag based localization.
 *
 * For an introduction to AprilTags, see the FTC-DOCS link below:
 * https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_intro/apriltag-intro.html
 *
 * In this sample, any visible tag ID will be detected and displayed, but only tags that are included in the default
 * "TagLibrary" will be used to compute the robot's location and orientation.  This default TagLibrary contains
 * the current Season's AprilTags and a small set of "test Tags" in the high number range.
 *
 * When an AprilTag in the TagLibrary is detected, the SDK provides location and orientation of the robot, relative to the field origin.
 * This information is provided in the "robotPose" member of the returned "detection".
 *
 * To learn about the Field Coordinate System that is defined for FTC (and used by this OpMode), see the FTC-DOCS link below:
 * https://ftc-docs.firstinspires.org/en/latest/game_specific_resources/field_coordinate_system/field-coordinate-system.html
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */

public class AprilDriver {
  private OpMode myOp = null;

  public AprilDriver(OpMode opmode) {
    myOp = opmode;
  }

  double kdist = .95; // 1.395

  protected double UnkId = 0.0;
  double UnkX = 0.0;
  double UnkY = 0.0;

  public static int MetaId = 0;
  public static String MetaName = "";
  protected double Rrange = 0.0;
  protected double Rpitch = 0.0;
  protected double Brange = 0.0;
  protected double Bpitch = 0.0;
  public static double range = 0.0;
  public static double bearing = 0.0;
  public static double actualBearing;
  public static double poseX;
  public static double poseY;

  public static final boolean USE_WEBCAM = true; // true for webcam, false for phone camera

  /**
   * Variables to store the position and orientation of the camera on the robot. Setting these
   * values requires a definition of the axes of the camera and robot:
   *
   * <p>Camera axes: Origin location: Center of the lens Axes orientation: +x right, +y down, +z
   * forward (from camera's perspective)
   *
   * <p>Robot axes (this is typical, but you can define this however you want): Origin location:
   * Center of the robot at field height Axes orientation: +x right, +y forward, +z upward
   *
   * <p>Position: If all values are zero (no translation), that implies the camera is at the center
   * of the robot. Suppose your camera is positioned 5 inches to the left, 7 inches forward, and 12
   * inches above the ground - you would need to set the position to (-5, 7, 12).
   *
   * <p>Orientation: If all values are zero (no rotation), that implies the camera is pointing
   * straight up. In most cases, you'll need to set the pitch to -90 degrees (rotation about the
   * x-axis), meaning the camera is horizontal. Use a yaw of 0 if the camera is pointing forwards,
   * +90 degrees if it's pointing straight left, -90 degrees for straight right, etc. You can also
   * set the roll to +/-90 degrees if it's vertical, or 180 degrees if it's upside-down.
   */
  private final Position cameraPosition = new Position(DistanceUnit.INCH, 0, 0, 0, 0);

  private final YawPitchRollAngles cameraOrientation =
      new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0);

  /** The variable to store our instance of the AprilTag processor. */
  private AprilTagProcessor aprilTag;

  /** The variable to store our instance of the vision portal. */
  private VisionPortal visionPortal;

  public void stopStream() {
    // Save CPU resources; can resume streaming when needed.
    visionPortal.stopStreaming();
  }

  public void startStream() {
    visionPortal.resumeStreaming();
  }

  public void closePortal() {
    // Save more CPU resources when camera is no longer needed.
    visionPortal.close();
  } // end method runOpMode()

  /** Initialize the AprilTag processor. */
  public void initAprilTag() {

    // Create the AprilTag processor.
    aprilTag =
        new AprilTagProcessor.Builder()

            // The following default settings are available to un-comment and edit as needed.
            // .setDrawAxes(false)
            // .setDrawCubeProjection(false)
            // .setDrawTagOutline(true)
            .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
            .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
            .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
            .setCameraPose(cameraPosition, cameraOrientation)
            // == CAMERA CALIBRATION ==
            // If you do not manually specify calibration parameters, the SDK will attempt
            // to load a predefined calibration for your camera.
            // .setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
            .setLensIntrinsics(823.189, 823.181, 388.908, 2234.377)
            // ... these parameters are fx, fy, cx, cy.
            .build();

    // Adjust Image Decimation to trade-off detection-range for detection-rate.
    // eg: Some typical detection data using a Logitech C920 WebCam
    // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
    // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
    // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
    // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
    // Note: Decimation can be changed on-the-fly to adapt during a match.
    // aprilTag.setDecimation(3);

    // Create the vision portal by using a builder.
    VisionPortal.Builder builder = new VisionPortal.Builder();

    // Set the camera (webcam vs. built-in RC phone camera).
    if (USE_WEBCAM) {
      builder.setCamera(myOp.hardwareMap.get(WebcamName.class, "Webcam 1"));
    } else {
      builder.setCamera(BuiltinCameraDirection.BACK);
    }

    // Choose a camera resolution. Not all cameras support all resolutions.
    // builder.setCameraResolution(new Size(640, 480));

    // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
    // builder.enableLiveView(true);

    // Set the stream format; MJPEG uses less bandwidth than default YUY2.
    // builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

    // Choose whether or not LiveView stops if no processors are enabled.
    // If set "true", monitor shows solid orange screen if no processors enabled.
    // If set "false", monitor shows camera view without annotations.
    // builder.setAutoStopLiveView(false);

    // Set and enable the processor.
    builder.addProcessor(aprilTag);

    // Build the Vision Portal, using the above settings.
    visionPortal = builder.build();

    // Disable or re-enable the aprilTag processor at any time.
    // visionPortal.setProcessorEnabled(aprilTag, true);

  } // end method initAprilTag()

  /** Add telemetry about AprilTag detections. */
  boolean redAprilTag = false;

  boolean blueAprilTag = false;

  public void getAprilTag() {
    boolean notRedBlue = true;
    List<AprilTagDetection> currentDetections = aprilTag.getDetections();

    // Step through the list of detections and display info for each one.
    for (AprilTagDetection detection : currentDetections) {
      if (detection.metadata != null) {
        MetaId = detection.id;
        MetaName = detection.metadata.name;
        if (detection.id == 24) {
          Rpitch = detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES);
          Rrange = detection.ftcPose.range * kdist; // distance constant to correct range
          notRedBlue = false;
          redAprilTag = true;
        } else {
          redAprilTag = false;
        }
        if (detection.id == 20) {
          Bpitch = detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES);
          Brange = detection.ftcPose.range * kdist; // distance constant to correct range
          notRedBlue = false;
          blueAprilTag = true;
        } else {
          blueAprilTag = false;
        }
        range = detection.ftcPose.range * kdist;
        bearing = detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES);
        actualBearing = detection.ftcPose.bearing;
        poseX = detection.ftcPose.x;
        poseY = detection.ftcPose.y;

        if (notRedBlue) {
          UnkId = detection.id;
          UnkX = detection.center.x;
          UnkY = detection.center.y;
        }
      } else {
        MetaId = 0;
      }
    } // end for() loop
  } // end method telemetryAprilTag()

  // allows range to be accessed
  public double getRange() {
    getAprilTag();
    return range;
  }

  public double getBearing() {
    getAprilTag();
    return bearing;
  }

  public double getActualBearing() {
    getAprilTag();
    return actualBearing;
  }

  // allows range to be accessed
  public int getMetaId() {
    getAprilTag();
    return MetaId;
  }

  public boolean getIsRedAprilTag() {
    getAprilTag();
    return redAprilTag;
  }

  public boolean getIsBlueAprilTag() {
    getAprilTag();
    return blueAprilTag;
  }

  public double getBearingAprilTag() {
    getAprilTag();
    return getActualBearing();
  }

  double fieldXHolonomic;
  double fieldYHolonomic;

  public double getFieldX(double heading) {
    double adjustedHeading = Math.toRadians(heading);
    double x = poseX * 2.54;
    double y = poseY * 2.54;

    fieldXHolonomic = -((x * Math.cos(adjustedHeading)) - (y * Math.sin(adjustedHeading))) - 146;
    return fieldXHolonomic;
  }

  public double getFieldY(double heading) {
    double adjustedHeading = Math.toRadians(heading);
    double x = poseX * 2.54;
    double y = poseY * 2.54;
    fieldYHolonomic = -(((x * Math.cos(adjustedHeading) + y * Math.sin(adjustedHeading))) - 146);
    return fieldYHolonomic;
  }
} // end class
