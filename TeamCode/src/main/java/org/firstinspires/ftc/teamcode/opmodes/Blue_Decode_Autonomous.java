// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.opmodes;

/* Copyright (c) 2024-2025 FTC 13532
-- All rights reserved.
- - - - -  eaglebots configuration - - - - - - - -
Left   Motor  0 . . . . . . . . .  LFM
Right  Motor  1 . . . . . . . . .  RFM
Left   Servo  0 . . . . . . . . .  LFS
Right  Servo  1 . . . . . . . . .  RFS

Left   Analog  . . . . .  Analog 0 LFP [one cable
Right  Analog  . . . . . .Analog 1 RFP  for both]
armpot Analog  . . . . . .Analog 2 armPot

Arm    Motor  2 . . . . . . . . . . arm
Spin   Motor  3 . . . . . . . . . . launch
Gate   Servo  4 . . . . . . . . . . gate
Intake Servo  2 . . . . . . . . . . intake
Guide  Servo  3 . . . . . . . . . . guide

Ball   Rev Color Sensor . . . IC1 0 ballSensor
Ball 1 ???
Ball 2 ???
Husky Camera   . . . . . . . .IC2 0 huskyLens
PinPoint . . . . . . . . . . .IC3 0 odo
April Camera   . . . . . . . . . . Webcam 1
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Decode.DC_Intake_Launch;
import org.firstinspires.ftc.teamcode.drivers.AprilDriver;

@Autonomous
public class Blue_Decode_Autonomous extends LinearOpMode {

  // Swerve Devices
  // -----------------------------
  // DC_Swerve_Drive drive = new DC_Swerve_Drive(this);
  // pinpoint
  // DC_Odometry_Sensor odo = new DC_Odometry_Sensor(this);
  // Game devices
  // -----------------------------
  // decode motors/servos
  private DcMotorEx driveLF = null;
  private DcMotorEx driveRT = null;
  private Servo servoLF = null;
  private Servo servoRT = null;
  private AnalogInput potLF = null;
  private AnalogInput potRT = null;
  DC_Intake_Launch decode = new DC_Intake_Launch(this);
  AprilDriver april = new AprilDriver(this);
  ElapsedTime runTime = new ElapsedTime();

  int side = 20; // Blue

  @Override
  public void runOpMode() {
    driveLF = (DcMotorEx) hardwareMap.dcMotor.get("LFM");
    driveRT = (DcMotorEx) hardwareMap.dcMotor.get("RFM");
    // driveLF.setDirection(DcMotorSimple.Direction.REVERSE);
    servoLF = hardwareMap.servo.get("LFS");
    servoRT = hardwareMap.servo.get("RFS");
    potLF = hardwareMap.get(AnalogInput.class, "LFP");
    potRT = hardwareMap.get(AnalogInput.class, "RFP");

    telemetry.addLine("Set hood height at 7 inches");
    telemetry.addLine("Fly wheel velocity 1900");
    waitForStart();
    try {

      // Initialize class components
      // drive.init();
      // odo.DoInit();
      decode.InitIL();
      april.initAprilTag();
      april.getAprilTag();
      if (april.MetaId == 24) side = 24;
      else side = 20;

      //  decode.runToHomePos(decode.chuteMotor, decode.home,decode.pot);

      // Wait for the DS start button to be touched.
      telemetry.addLine("Autonomous Ready:" + side);
      telemetry.update();
      // set angle
      /*
      april.getAprilTag();
      sleep(100); // wait for april response
      // decode.chuteAngle(april.range); // range in inches
      decode.spinUp(april.range); // start flywheel to reduce current

      // adjust for bearing, may be able to remove this adjust code
      while (opModeIsActive()) {
        april.getAprilTag();
        double bshift = april.range * Math.sin(Math.toRadians(april.bearing));
        runTime.reset();
        while (opModeIsActive() && runTime.seconds() < 3.0 && april.bearing > 1.0) {
          april.getAprilTag();
          // align robot to april with field oriented movements
          double pshift = april.range * Math.sin(Math.toRadians(april.bearing));
          double sshift =
              pshift / bshift; // move right or left to center bearing, May need to reverse
          // if (side == 24) sshift = -sshift; // move left if read
          drive.fieldRelativeDrive(
              -0.0 * drive.maxSpeedMetersPerSec,
              -sshift * drive.maxSpeedMetersPerSec,
              -0.0 * drive.maxOmegaRadPerSec);
        } // end while adjust for bearing

         */
      decode.spinUp(1900);
      sleep(1000);
      // shoot 3 balls
      decode.openGate();

      decode.Intake();
      sleep(2000); // 1.0 wait for ball to launch
      decode.IntakeStop();
      sleep(4000); // 2.0
      decode.Intake();
      sleep(2000);
      decode.IntakeStop();
      sleep(2000); // 3.0
      decode.Intake();

      decode.closeGate();
      decode.spinOff();

      servoLF.setPosition(0.5);
      servoRT.setPosition(0.5);

      sleep(1000);
      driveRT.setPower(+.3);
      driveLF.setPower(-.4);

      sleep(1080);
      driveRT.setPower(-1.0);
      driveLF.setPower(-1.0);

    } // end try
    catch (Exception e) {
      telemetry.addLine(", exception in gamePadTeleOP");
      telemetry.update();
      sleep(2200);
      requestOpModeStop();
    } // catch exception
  } // run op mode
} // end class Swerve Components
