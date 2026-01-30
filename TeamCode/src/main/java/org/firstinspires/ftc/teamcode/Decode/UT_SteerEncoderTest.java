// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Decode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Decode.wpilib.geometry.Rotation2d;

@TeleOp
public class UT_SteerEncoderTest extends LinearOpMode {
  @Override
  public void runOpMode() throws InterruptedException {
    var steerServos = new Servo[2];
    var encoders = new AnalogInput[2];
    var offsets = new Rotation2d[] {Rotation2d.fromDegrees(-2.5), Rotation2d.fromDegrees(-5)};
    steerServos[0] = hardwareMap.servo.get("LFS");
    steerServos[1] = hardwareMap.servo.get("RFS");
    encoders[0] = hardwareMap.get(AnalogInput.class, "LFP");
    encoders[1] = hardwareMap.get(AnalogInput.class, "RFP");
    waitForStart();
    var lastTimeStamp = System.nanoTime() / 1e9;
    while (opModeIsActive()) {
      double currentTime = System.nanoTime() / 1e9;
      double dt = currentTime - lastTimeStamp;
      for (int i = 0; i < 2; i++) {
        var currentAngle =
            Rotation2d.fromRotations(-encoders[i].getVoltage() / encoders[i].getMaxVoltage())
                .plus(offsets[i]);
        steerServos[i].setPosition(
            calculateSteerPID(Rotation2d.kZero.minus(currentAngle), i, dt) / 2 + .5);
        telemetry.addData("Wheel " + i + " currentAngle", currentAngle.getDegrees());
      }
      lastTimeStamp = currentTime;
      telemetry.update();
    }
  }

  private final double[] lastErrorRad = new double[2];

  private double calculateSteerPID(Rotation2d angleError, int i, double dt) {
    double errorRad = angleError.getRadians();
    double kP = .6 / (Math.PI / 2);
    double kD = 0.00;
    double kS = .035;
    double proportional = errorRad * kP;
    double derivative = kD * (errorRad - lastErrorRad[i]) / dt;
    telemetry.addData("Wheel " + i + " proportional", proportional);
    telemetry.addData("Wheel " + i + " derivative", derivative);
    lastErrorRad[i] = errorRad;
    var output = proportional + derivative;
    return output + kS * Math.signum(output);
  }
}
