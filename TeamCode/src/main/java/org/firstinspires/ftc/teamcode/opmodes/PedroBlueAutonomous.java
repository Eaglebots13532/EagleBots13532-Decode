// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.opmodes;

import static com.pedropathing.ivy.Scheduler.*;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.*;
import static com.pedropathing.ivy.pedro.PedroCommands.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.Decode.CarlHoodShoot;
import org.firstinspires.ftc.teamcode.drivers.AprilDriver;
import org.firstinspires.ftc.teamcode.drivers.GameDriver;
import org.firstinspires.ftc.teamcode.drivers.odo.CarlOdometryExampleImplementation;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Pedro Blue Autonomous", group = "Autonomous")
public class PedroBlueAutonomous extends OpMode {

  private GameDriver gameDriver;

  private Follower follower;
  private final Pose startPose =
      new Pose(0, 0, Math.toRadians(0)); // Starting position of the robot
  private final Pose scorePose = new Pose(8, -1.8, 1.85); // Position to score the game element
  private final Pose parkPose = new Pose(16.3, 23.8, 0); // Position to park after scoring

  private PathChain scorePath, parkPath;
  private CarlHoodShoot hood;
  private CarlOdometryExampleImplementation odo;
  private AprilDriver april;

  public void buildPaths() {

    scorePath =
        follower
            .pathBuilder()
            .addPath(new BezierLine(startPose, scorePose))
            .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
            .build();

    parkPath =
        follower
            .pathBuilder()
            .addPath(new BezierLine(scorePose, parkPose))
            .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
            .build();
  }

  public Command autoRoutine() {
    return sequential(
        parallel(follow(follower, scorePath), instant(() -> gameDriver.setLaunchPower(1400))),
        instant(() -> gameDriver.openGate()),
        waitMs(500),
        parallel(
            follow(follower, parkPath),
            instant(() -> gameDriver.closeGate()),
            instant(() -> gameDriver.setLaunchPower(0))));
  }

  @Override
  public void start() {
    schedule(autoRoutine());
  }

  @Override
  public void init() {
    gameDriver = new GameDriver(hardwareMap, telemetry);

    Scheduler.reset();
    follower = Constants.createFollower(hardwareMap);
    buildPaths();
    follower.setStartingPose(startPose);

    april = new AprilDriver(this);
    odo = new CarlOdometryExampleImplementation(april);
    hood = new CarlHoodShoot(gameDriver, odo);
  }

  @Override
  public void loop() {
    follower.update();
    Scheduler.execute();
    // Feedback to Driver Hub for debugging
    telemetry.addData("x", follower.getPose().getX());
    telemetry.addData("y", follower.getPose().getY());
    telemetry.addData("heading", follower.getPose().getHeading());
    telemetry.update();
  }
}
