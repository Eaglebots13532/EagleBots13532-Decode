package org.firstinspires.ftc.teamcode.Slide;

import static org.firstinspires.ftc.teamcode.ODO.GoBildaPinpointDriver.EncoderDirection.FORWARD;
import static org.firstinspires.ftc.teamcode.ODO.GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.ODO.GoBildaPinpointDriver;

public class Arm {

  public DcMotor arm_Len, rEXT, lEXT, pivot;
  private LinearOpMode opMode;
  public double offset = 0.0;

  public Arm(LinearOpMode opMode) {
    this.opMode = opMode;
    arm_Len = opMode.hardwareMap.get(DcMotor.class, "Arm odometry");
    arm_Len.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    arm_Len.setDirection(DcMotorSimple.Direction.REVERSE);
    offset = arm_Len.getCurrentPosition();
    rEXT = opMode.hardwareMap.get(DcMotor.class, "right extension");
    rEXT.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    rEXT.setDirection(DcMotorSimple.Direction.REVERSE);
    rEXT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    lEXT = opMode.hardwareMap.get(DcMotor.class, "left extension");
    lEXT.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    lEXT.setDirection(DcMotorSimple.Direction.FORWARD);
    lEXT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    pivot = opMode.hardwareMap.get(DcMotor.class, "pivot");
    pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    pivot.setDirection(DcMotorSimple.Direction.FORWARD);
    pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
  }

  public void set_Arm_Pos(double len, double ang) {

  }

  public void set_Arm_Ang(double ang) {
    pivot.setPower(ang);
  }

  public void set_Arm_Len(double len) {
    rEXT.setPower(len);
    lEXT.setPower(len);
    pivot.setPower(-len/1.5);
  }

  public double get_Arm_Len() { // 12in is roughly from -199 to -56__ 
    double len = offset;
    len += arm_Len.getCurrentPosition();
    opMode.telemetry.addLine("Arm pos: " + len);
    opMode.telemetry.addLine("Offset: " + offset);
    return len;
  }

  public double get_Arm_Ang() {
    double ang = 0;

    return ang;
  }
}
