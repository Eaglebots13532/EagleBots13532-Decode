// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.ODO.GoBildaPinpointDriver.EncoderDirection.REVERSED;
import static org.firstinspires.ftc.teamcode.ODO.GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Mekanism.Mekanism;
import org.firstinspires.ftc.teamcode.ODO.GoBildaPinpointDriver;

@Autonomous(name = "output XY position")
public class OutputXYpos extends LinearOpMode {
    private GoBildaPinpointDriver odometry;
    private Mekanism mek;

    @Override
    public void runOpMode() throws InterruptedException {
        odometry = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odometry.resetPosAndIMU();sleep(250);
        odometry.setOffsets(110, 30);
        sleep(100);
        odometry.setEncoderResolution(goBILDA_4_BAR_POD);
        odometry.setEncoderDirections(REVERSED, REVERSED);
        odometry.resetHeading();

        waitForStart();
        while (opModeIsActive()) {
            odometry.update();
            telemetry.addData("Heading:", odometry.getHeading());
            telemetry.addData("  X pos:", odometry.getPosX());
            telemetry.addData("  Y pos:", odometry.getPosY());
            telemetry.update();
        }// While opmode active
    }// run Op Mode
}// OutputXY end
