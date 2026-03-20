// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Decode;

/* March 7, 2026
 * A test jig is being used to develop the concept of running the decode:hood by encoder
 * The goal is to simplify the hood software and make it more usable.
 * The Hood will have a homing switch to provide an encoder reference. From this reference
 * the hood will move to the desired height using the range to the april tag, provided by
 * the web camera.
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.drivers.udpwifiData;
import org.firstinspires.ftc.teamcode.drivers.wpilib.interpolation.InterpolatingDoubleTreeMap;

@TeleOp(name = "Hood Encoder Basics")
public class HoodEncoder extends LinearOpMode {
  private DcMotorEx motor = null;
  private CRServo servo = null;
  public TouchSensor bob = null;
  int homeEncoder = 0;

  int homedirection = 1; // neg 1 for - power to home
  int currentPosition = 0;
  int targetPosition = 0;

  udpwifiData sndData = new udpwifiData();

  private final ElapsedTime eTime = new ElapsedTime();
  private final ElapsedTime dTime = new ElapsedTime();

  // servo power and pid error
  private static double servode = 0.0;
  private static final int revCtperRev = 8192;
  private static final double gearDia = 1.415;
  private static final double gearCircum = gearDia * 3.1415;

  public void runOpMode() {

    servo = hardwareMap.get(CRServo.class, "Servo");
    servo.setDirection(DcMotorSimple.Direction.FORWARD);
    // enable encoder calls - no motor
    motor = hardwareMap.get(DcMotorEx.class, "Motor");
    bob = hardwareMap.get(TouchSensor.class, "TouchSensor");
    dTime.reset();
    waitForStart();
    try {
      GotoHome();
      servo.setPower(-.8);
      homeEncoder = motor.getCurrentPosition();
      int target = -8191;
      while (opModeIsActive() && target < motor.getCurrentPosition()) {
        telemetry.addLine("Time: " + (dTime.nanoseconds()) / 1e9);
        telemetry.addLine("position: " + motor.getCurrentPosition());
        telemetry.update();
      }
      motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      servo.setPower(0.0);
      telemetry.addLine("position: " + motor.getCurrentPosition());
      telemetry.update();
      sleep(5000);

    } catch (Exception e) {
      e.printStackTrace();
    }
  } // ends runOpmode

  public void HoodPosition(int newPosition) {
    try {
      sndData.sendData(eTime.milliseconds(), 1, (double) targetPosition);
      // target position!
      dTime.reset(); // differential reset
      eTime.reset(); // udp reset
      while (opModeIsActive() && IsBusy(newPosition)) {
        idle();
      }
      servo.setPower(0.0);

    } catch (Exception e) {
      e.printStackTrace();
    }
  } // end hood position

  // range is the camera range in inches from target or gamepad -1 to 1 converted to 20 to 130
  // inches
  public int EncCntfrmRange(double range) {
    InterpolatingDoubleTreeMap distanceToHoodMap = new InterpolatingDoubleTreeMap();
    distanceToHoodMap.put(43.0, 2.0);
    distanceToHoodMap.put(79.0, 4.0);
    distanceToHoodMap.put(126.0, 8.0);
    distanceToHoodMap.put(130.0, 8.5);
    distanceToHoodMap.put(160.0, 9.5);
    distanceToHoodMap.put(296.0, 10.5);
    final double Hm = 10.5; // maximum hood height
    final int Hmc = (int) (revCtperRev * Hm / gearCircum); // max hood encoder count
    double Hh = distanceToHoodMap.get(range); // desired hood height from range
    int He = (int) (revCtperRev * Hh / gearCircum); // hood desired count position
    return He;
  }

  private boolean IsBusy(int target) {
    final double k = 0.01;
    final double d = .005;
    int encCt = motor.getCurrentPosition();
    int sgn = target > encCt ? -1 : 1; // change signs for reverse movement
    double tdiff = encCt - target;
    double servoPwr = sgn * tdiff * k + d * (tdiff - servode) / dTime.milliseconds();
    sndData.sendData(eTime.milliseconds(), 1, (double) servoPwr);
    servo.setPower(Range.clip(servoPwr, -1, 1));
    servode = tdiff; // save error for differential error
    dTime.reset(); // reset for differential
    return !(tdiff < 1);
  }

  // Move hood to home if not already there and reset encoder to 0
  public void GotoHome() {
    double hmpwr = .2;
    if (hmpwr > 0) homedirection = 1;
    else homedirection = -1;
    servo.setPower(hmpwr);
    while (opModeIsActive() && !bob.isPressed()) {
      idle();
    }
    servo.setPower(0.0);
    motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    homeEncoder = motor.getCurrentPosition();
  }
}
