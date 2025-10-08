// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.AprilTags;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;
public class AprilTagXYZ {

    // Default constructor
    public AprilTagXYZ (LinearOpMode opmode) {
        myOp = opmode;
    }
    /* Declare OpMode members.
     * gain access to methods in the calling OpMode.
     */
    private LinearOpMode myOp = null;

    /**
     * Team library tags for AprilTag processor.
     */
    AprilTagLibrary myAprilTagLibrary;
    /**
     * The variable to store our instance of the AprilTag processor.
     */
    protected AprilTagProcessor aprilTag;
    protected VisionPortal visionPortal;
    private double tagX = 0.0, tagY = 0.0, tagZ = 0.0;
    private double angP = 0.0, angR = 0.0, angY = 0.0;
    private String tagName = "";
    private int    tagNumb = 0;

    // Camera objects
    private Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 0, 0, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);

    /**
     * Add telemetry about AprilTag detections.
     * may set the distance and angle to the tag
     * From a known april tag location provides reference to field orientation
     * Back projection to center of field - - -
     */
    public boolean aprilRecognize() {
        boolean found = false;
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        //int tagSize = currentDetections.size();
        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            found = true;
            if (detection.metadata != null) {
                tagNumb = detection.id;
                tagName = detection.metadata.name;
                // inches
                //tagX = detection.robotPose.getPosition().x;
                tagX = detection.ftcPose.x;
                //tagY = detection.robotPose.getPosition().y;
                tagY = detection.ftcPose.y;
                //tagZ = detection.robotPose.getPosition().z;
                tagZ = detection.ftcPose.z;
                angP = detection.robotPose.getOrientation().getPitch(AngleUnit.RADIANS);
                angR = detection.robotPose.getOrientation().getRoll(AngleUnit.RADIANS);
                //angY = detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS);
                angY = detection.ftcPose.yaw;
            } else {
                tagNumb = detection.id;
                tagX = detection.center.x;
                tagY = detection.center.y;
                tagZ = 0.0;
                found = true;
            }

        }   // end for() loop
        if(!found) {
            tagNumb = 0;
            tagName = "";
            tagX = 0.0;
            tagY = 0.0;
            angP = 0.0;
            angR = 0.0;
            angY = 0.0;
        }
        return found;
    }   // end method AprilRecognize()

    /**
     * Initialize the AprilTag processor. Make sure units are the same all around
     */
    public void initAprilTag() {
        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                .setDrawAxes(true)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setTagLibrary(AprilTagGameDatabase.getIntoTheDeepTagLibrary())
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
                .setCameraPose(cameraPosition, cameraOrientation)
                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.
                .setLensIntrinsics(813.078,813.078,331.785,254.835)

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        builder.setCamera(myOp.hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(640,480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(false);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()

    public double getAprilDstX(){
        return tagX;
    }
    public double getAprilDstY(){
        return tagY;
    }
    // return the April tag Yaw in radians
    public double getAprilYaw(){
        double angRad = Math.toRadians(angY);
        return angRad;
    }
    public double getTagRange(){
        return Math.sqrt(tagX * tagX + tagY * tagY);
    }

    public double getTagBearing(){
        return Math.tan(tagX/tagY);
    }

    public String getTagName(){
        return tagName;
    }
    public double getTagNumber(){
        return tagNumb;
    }

    protected void TeamTagFamily(){
        AprilTagMetadata myAprilTagMetadata;
        AprilTagLibrary.Builder myAprilTagLibraryBuilder;
        AprilTagProcessor.Builder myAprilTagProcessorBuilder;

        AprilTagProcessor myAprilTagProcessor;

// Create a new AprilTagLibrary.Builder object and assigns it to a variable.
        myAprilTagLibraryBuilder = new AprilTagLibrary.Builder();

// Add all the tags from the given AprilTagLibrary to the AprilTagLibrary.Builder.
// Get the AprilTagLibrary for the current season.
        myAprilTagLibraryBuilder.addTags(AprilTagGameDatabase.getCurrentGameTagLibrary());

// Create a new AprilTagMetdata object and assign it to a variable.
        myAprilTagMetadata = new AprilTagMetadata(4, "Position_4", 3.5, DistanceUnit.INCH);
        myAprilTagMetadata = new AprilTagMetadata(8, "Position_8", 3.5, DistanceUnit.INCH);

// Add a tag to the AprilTagLibrary.Builder.
        myAprilTagLibraryBuilder.addTag(myAprilTagMetadata);

// Build the AprilTag library and assign it to a variable.
        myAprilTagLibrary = myAprilTagLibraryBuilder.build();

// Create a new AprilTagProcessor.Builder object and assign it to a variable.
        myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();

// Set the tag library.
        myAprilTagProcessorBuilder.setTagLibrary(myAprilTagLibrary);

// Build the AprilTag processor and assign it to a variable.
        myAprilTagProcessor = myAprilTagProcessorBuilder.build();

    }
}
