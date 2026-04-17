// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.drivers.odo;

import android.util.Size;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class WebCamCarlCoaxSwerve {
  private AprilTagProcessor aprilTagProcessor;
  private VisionPortal visionPortal;
  private List<AprilTagDetection> detectedTags = new ArrayList<>();
  private Telemetry telemetry;

  public void init(HardwareMap hwMap) {

    aprilTagProcessor =
        new AprilTagProcessor.Builder()
            .setDrawTagID(true)
            .setDrawTagOutline(true)
            .setDrawAxes(true)
            .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
            .build();
    VisionPortal.Builder builder = new VisionPortal.Builder();
    builder.setCamera(hwMap.get(WebcamName.class, "Webcam 1"));
    builder.setCameraResolution(new Size(640, 480));
    builder.addProcessor(aprilTagProcessor);

    visionPortal = builder.build();
  }

  public void update() {
    detectedTags = aprilTagProcessor.getDetections();
  }

  public List<AprilTagDetection> getDetectedTags() {
    return detectedTags;
  }

  public AprilTagDetection getTagBySpecificId(int Id) {
    for (AprilTagDetection detection : detectedTags) {
      if (detection.id == Id) {
        return detection;
      }
    }
    return null;
  }

  public void stop() {
    if (visionPortal != null) {
      visionPortal.close();
    }
  }

  public void displayDetectionTelemetry(AprilTagDetection detectedId) {
    if (detectedId == null) {
      return;
    }
    if (detectedId.metadata != null) {
      telemetry.addLine(
          String.format("\n==== (ID %d) %s", detectedId.id, detectedId.metadata.name));
      telemetry.addLine(
          String.format(
              "XYZ %6.1f %6.1f %6.1f  (inch)",
              detectedId.ftcPose.x, detectedId.ftcPose.y, detectedId.ftcPose.z));
      telemetry.addLine(
          String.format(
              "PRY %6.1f %6.1f %6.1f  (deg)",
              detectedId.ftcPose.pitch, detectedId.ftcPose.roll, detectedId.ftcPose.yaw));
      telemetry.addLine(
          String.format(
              "RBE %6.1f %6.1f %6.1f  (inch, deg, deg)",
              detectedId.ftcPose.range, detectedId.ftcPose.bearing, detectedId.ftcPose.elevation));
    } else {
      telemetry.addLine(String.format("\n==== (ID %d) Unknown", detectedId.id));
      telemetry.addLine(
          String.format("Center %6.0f %6.0f   (pixels)", detectedId.center.x, detectedId.center.y));
    }

    // end for() loop
  }

  public Double getAprilDistance(int id) {
    update();
    AprilTagDetection tag = getTagBySpecificId(id);
    if (tag == null) return null;
    return tag.ftcPose.range;
  }

  public Double getAprilBearing(int id) {
    update();
    AprilTagDetection tag = getTagBySpecificId(id);
    if (tag == null) return null;
    return tag.ftcPose.bearing;
  }

  public AprilTagDetection getTag20or24() {
    update();
    for (AprilTagDetection tag : detectedTags) {
      if (tag.id == 20 || tag.id == 24) {
        return tag;
      }
    }
    return null;
  }

  public Double getAprilDistance20Or24() {
    update();
    AprilTagDetection tag = getTag20or24();
    if (tag == null) return null;
    return tag.ftcPose.range;
  }

  public Double getAprilBearing20Or24() {
    update();
    AprilTagDetection tag = getTag20or24();
    if (tag == null) return null;
    return tag.ftcPose.bearing;
  }
}
