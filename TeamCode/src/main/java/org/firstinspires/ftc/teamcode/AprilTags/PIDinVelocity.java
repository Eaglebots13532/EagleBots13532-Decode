// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.AprilTags;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
@Config
@Autonomous
public class PIDinVelocity extends LinearOpMode {

        // our DC motor.
        DcMotor BLmotor;

        public static final double NEW_P = 2.5;
        public static final double NEW_I = 0.1;
        public static final double NEW_D = 0.2;
        public static final double NEW_F = 0.2;

        public void runOpMode() {
            FtcDashboard dashboard = FtcDashboard.getInstance();
            telemetry = dashboard.getTelemetry();
            // get reference to DC motor.
           BLmotor = hardwareMap.get(DcMotor.class, "BLMotor");

            // wait for start command.
            waitForStart();

            // get a reference to the motor controller and cast it as an extended functionality controller.
            // we assume it's a REV Robotics Control Hub or REV Robotics Expansion Hub (which supports the extended controller functions).
            DcMotorControllerEx motorControllerEx = (DcMotorControllerEx)BLmotor.getController();

            // get the port number of our configured motor.
            int motorIndex = ((DcMotorEx)BLmotor).getPortNumber();

            // get the PID coefficients for the RUN_USING_ENCODER  modes.
            PIDFCoefficients pidOrig = motorControllerEx.getPIDFCoefficients(motorIndex,DcMotor.RunMode.RUN_USING_ENCODER);
            // change coefficients.
            PIDFCoefficients pidNew = new PIDFCoefficients(NEW_P,NEW_I,NEW_D,NEW_F);
            motorControllerEx.setPIDFCoefficients(motorIndex, DcMotor.RunMode.RUN_USING_ENCODER, pidNew);

            // re-read coefficients and verify change.
            PIDFCoefficients pidModified = motorControllerEx.getPIDFCoefficients(motorIndex, DcMotor.RunMode.RUN_USING_ENCODER);

            // display info to user.
            while(opModeIsActive()) {
                telemetry.addData("Runtime", "%.03f", getRuntime());
                telemetry.addData("P,I,D (orig)", "%.04f, %.04f, %.0f",
                        pidOrig.p, pidOrig.i, pidOrig.d);
                telemetry.addData("P,I,D (modified)", "%.04f, %.04f, %.04f",
                        pidModified.p, pidModified.i, pidModified.d);
                telemetry.update();
            }
        }
    }
