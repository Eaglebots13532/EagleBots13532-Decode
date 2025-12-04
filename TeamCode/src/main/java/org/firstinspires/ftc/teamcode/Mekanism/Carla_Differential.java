package org.firstinspires.ftc.teamcode.Mekanism;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

public class Carla_Differential {

  LinearOpMode opMode;
  public CRServo left, right;

  public Carla_Differential(LinearOpMode opMode) {
    this.opMode = opMode;
    left = opMode.hardwareMap.get(CRServo.class, "left differential");
    right = opMode.hardwareMap.get(CRServo.class, "right differential");

//    left.resetDeviceConfigurationForOpMode();
//    right.resetDeviceConfigurationForOpMode();

    left.setDirection(FORWARD);
    right.setDirection(REVERSE);

    left.setPower(0);
    right.setPower(0);
  }

  public void move(double pos, double rotation) {
    double left_diff = pos;
    double right_diff = pos;
    if (-.05 < pos && pos < 0.05) {
      left_diff = -rotation;
      right_diff = rotation;
    }
    left.setPower(left_diff);
    right.setPower(right_diff);
  }
}
