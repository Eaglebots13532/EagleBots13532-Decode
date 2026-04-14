// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.CoaxialPod;
import com.pedropathing.ftc.drivetrains.SwerveConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

  private static double dtLength = 2.75;
  private static double dtWidth = 14.5;
  public static FollowerConstants followerConstants =
      new FollowerConstants()
          .mass(13.6)
          // .centripetalScaling(0.0005)
          .forwardZeroPowerAcceleration(-90)
          .lateralZeroPowerAcceleration(-90)
          .translationalPIDFCoefficients(new PIDFCoefficients(0.35, 0, 0.03, 0.1))
          .headingPIDFCoefficients(new PIDFCoefficients(1.2, 0, 0.1, 0.05))
      // .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.005, 0, 0.00003, 0.6, 0.13))
      // .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(
      // 0.05, //0.05 to 0.3
      // 0,//0.38735914623969386,
      // 0.002)
      // )
      ;
  public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

  public static SwerveConstants swerveConstants =
      new SwerveConstants()
          .maxPower(.8) // determines the max power of the drivetrain
          // .zeroPowerBehavior(SwerveConstants.ZeroPowerBehavior.IGNORE_ANGLE_CHANGES);

          .zeroPowerBehavior(SwerveConstants.ZeroPowerBehavior.X_LOCK)
          .velocity(15.6);

  // the above disables x locking for swerve, which can be useful for tuning pod offsets

  private static CoaxialPod leftFront(HardwareMap hardwareMap) {
    CoaxialPod pod =
        new CoaxialPod(
            hardwareMap,
            "LFM", // the name of your motor in your config
            "LFS", // the name of your servo in your config
            "LFP", // the name of your analog encoder in your config
            new PIDFCoefficients(0.3, 0, 0.005, 0.01), // pod PIDF coefficients
            DcMotorSimple.Direction.FORWARD, // the direction of your motor
            DcMotorSimple.Direction.REVERSE, // the direction of your servo
            Math.toRadians(182.0), // your pod's angle offset, in radians
            new Pose(-dtLength, dtWidth), // your pods x and y offsets,
            // in pedro coordinates (like with deadwheels)
            0.0, // analog min voltage
            3.289, // analog max voltage
            false); // encoder inverted
    //  uncom.0ment the below lines to change caching thresholds (by default 0.01)
    //  pod.setMotorCachingThreshold(0.05);
    //  pod.setServoCachingThreshold(0.05);
    return pod;
  }

  private static CoaxialPod rightFront(HardwareMap hardwareMap) {
    CoaxialPod pod =
        new CoaxialPod(
            hardwareMap,
            "RFM", // the name of your motor in your config
            "RFS", // the name of your servo in your config
            "RFP", // the name of your analog encoder in your config
            new PIDFCoefficients(0.35, 0, 0.005, 0.01), // pod PIDF coefficients
            DcMotorSimple.Direction.REVERSE, // the direction of your motor
            DcMotorSimple.Direction.REVERSE, // the direction of your servo
            Math.toRadians(116.2), // your pod's angle offset, in radians
            new Pose(-dtLength, -dtWidth), // your pods x and y offsets,
            // in pedro coordinates (like with deadwheels)
            0.002, // analog min voltage
            3.275, // analog max voltage
            false); // encoder inverted
    //  uncom.0ment the below lines to change caching thresholds (by default 0.01)
    //  pod.setMotorCachingThreshold(0.05);
    //  pod.setServoCachingThreshold(0.05);
    return pod;
  }

  public static PinpointConstants localizerConstants =
      new PinpointConstants()
          .forwardPodY(6)
          .strafePodX(-4.5)
          .distanceUnit(DistanceUnit.INCH)
          .hardwareMapName("odo")
          .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
          .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
          .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

  public static Follower createFollower(HardwareMap hardwareMap) {
    return new FollowerBuilder(followerConstants, hardwareMap)
        .pathConstraints(pathConstraints)
        .swerveDrivetrain(swerveConstants, leftFront(hardwareMap), rightFront(hardwareMap))
        .pinpointLocalizer(localizerConstants)
        .build();
  }
}
