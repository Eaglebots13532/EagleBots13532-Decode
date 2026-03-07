package org.firstinspires.ftc.teamcode.cole;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Tank Drive With Triggers", group = "TeleOp")
public class Cole extends LinearOpMode {

    private DcMotor leftFront;
    private DcMotor leftRear;
    private DcMotor rightFront;
    private DcMotor rightRear;


    public void runOpMode() {

        // Hardware mapping
        leftFront  = hardwareMap.get(DcMotor.class, "LMotor");
        leftRear   = hardwareMap.get(DcMotor.class, "BMotor");
        rightFront = hardwareMap.get(DcMotor.class, "RMotor");
        rightRear  = hardwareMap.get(DcMotor.class, "FMotor");

        // Reverse left side if needed
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {

            // Stick control (Tank Drive)
            double leftPower  = -gamepad1.left_stick_y;
            double rightPower = -gamepad1.right_stick_y;

            // Trigger override
            double forward  = gamepad1.right_trigger;
            double backward = gamepad1.left_trigger;

            if (forward > 0.05) {
                leftPower = forward;
                rightPower = forward;
            }
            else if (backward > 0.05) {
                leftPower = -backward;
                rightPower = -backward;
            }

            // Set motor power
            leftFront.setPower(leftPower);
            leftRear.setPower(leftPower);
            rightFront.setPower(rightPower);
            rightRear.setPower(rightPower);

            telemetry.addData("Left Power", leftPower);
            telemetry.addData("Right Power", rightPower);
            telemetry.update();//
        }
    }
}