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
  private final Pose shootPose = new Pose(14.5, -1.8, 2.05); // Position to score the game element
  private final Pose toBallsPose = new Pose(40.2, 8.0, 0.90); // Go to the line where balls are set up
  private final Pose getBallsPose = new Pose(40, 30, 1.60); // Pick up three balls while intaking


  private PathChain scorePath, ballPath, parkPath, intakePath;
  private CarlHoodShoot hood;
  private CarlOdometryExampleImplementation odo;
  private AprilDriver april;
  boolean isHoodHoming = true;
  boolean isArmHoming = true;
  double seekHoodAngle;
  double FlyVel;


  public void buildPaths() {

    scorePath =
        follower
            .pathBuilder()
            .addPath(new BezierLine(startPose, shootPose))
            .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
            .build();

    ballPath =
        follower
            .pathBuilder()
            .addPath(new BezierLine(shootPose, toBallsPose))
            .setLinearHeadingInterpolation(shootPose.getHeading(), toBallsPose.getHeading())
            .build();
    intakePath =
            follower
                    .pathBuilder()
                    .addPath(new BezierLine(toBallsPose, getBallsPose))
                    .setLinearHeadingInterpolation(toBallsPose.getHeading(), getBallsPose.getHeading())
                    .build();
    parkPath =
            follower
                    .pathBuilder()
                    .addPath(new BezierLine(shootPose, toBallsPose))
                    .setLinearHeadingInterpolation(shootPose.getHeading(), toBallsPose.getHeading())
                    .build();
  }// finishes buildPaths




  public Command autoRoutine() {
    return sequential(
        parallel(
                follow(follower, scorePath),
                instant(() -> gameDriver.setLaunchPower(1400))),

        instant(() -> gameDriver.openGate()),
            waitMs(5000),// shoot three balls

        follow(follower,ballPath),
        //go to the balls

        parallel(
                follow(follower,intakePath),
                instant(()-> gameDriver.intakeOn())),
                //get balls intake etc

        parallel(

                follow(follower, scorePath),
                instant(() -> gameDriver.setLaunchPower(1400)),

                instant(() -> gameDriver.openGate()),
                waitMs(5000)), // go back to the shoot position, shoot the balls again

        parallel(
            follow(follower, parkPath),
            instant(() -> gameDriver.closeGate()),
            instant(() -> gameDriver.setLaunchPower(0)))); //leave and stop shooting
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
    if (isHoodHoming) {
      hood.hoodHome();
      isHoodHoming = hood.hasHomed();
    }
    if (isArmHoming) {
      gameDriver.homeArm();
      isArmHoming = gameDriver.getArmHomeSensor();
    }
    gameDriver.setHoodServoPower(hood.getHoodServoPowerPID(seekHoodAngle, getRuntime()));
    gameDriver.setFlyVelRPM(FlyVel);
    //hood.getAngleError() use for sequential control
  }
}
