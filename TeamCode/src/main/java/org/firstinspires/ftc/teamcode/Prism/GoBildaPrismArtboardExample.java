// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Prism;

import static org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver.Artboard;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.concurrent.TimeUnit;

/*
 * This example shows how to recall previously created Artboards on the goBILDA Prism RGB Driver.
 * Recalling these animations is all we recommend that your main robot OpMode do. We prefer to
 * create these Artboards once and store them on the device, allowing for a short I²C write to
 * set the current Artboard.
 *
 * It also shows how to enable or disable the default boot animation on power up. When enabled,
 * as soon as the Prism gets power it will display the Artboard saved in Artboard slot 0. If you'd
 * instead like to wait until you explicitly set an Artboard, disable this setting. This setting is
 * saved and will only need to be set once.
 *
 */
@TeleOp(name = "Prism Artboard Example", group = "Linear OpMode")
// @Disabled

public class GoBildaPrismArtboardExample extends LinearOpMode {

  GoBildaPrismDriver prism;

  @Override
  public void runOpMode() {

    /*
     * Initialize the hardware variables. Note that the strings used here must correspond
     * to the names assigned during the robot configuration step on the driver's station.
     */
    prism = hardwareMap.get(GoBildaPrismDriver.class, "prism");

    telemetry.addData("Device ID: ", prism.getDeviceID());
    telemetry.addData("Firmware Version: ", prism.getFirmwareVersionString());
    telemetry.addData("Hardware Version: ", prism.getHardwareVersionString());
    telemetry.addData("Power Cycle Count: ", prism.getPowerCycleCount());
    telemetry.addData("Run Time (Minutes): ", prism.getRunTime(TimeUnit.MINUTES));
    telemetry.update();

    // Wait for the game to start (driver presses START)
    waitForStart();
    resetRuntime();

    // run until the end of the match (driver presses STOP)
    while (opModeIsActive()) {

      if (gamepad1.aWasPressed()) {
        prism.loadAnimationsFromArtboard(Artboard.ARTBOARD_0);
      } else if (gamepad1.bWasPressed()) {
        prism.loadAnimationsFromArtboard(Artboard.ARTBOARD_1);
      } else if (gamepad1.yWasPressed()) {
        prism.loadAnimationsFromArtboard(Artboard.ARTBOARD_2);
      } else if (gamepad1.xWasPressed()) {
        prism.loadAnimationsFromArtboard(Artboard.ARTBOARD_3);
      }

      if (gamepad1.dpadLeftWasPressed()) {
        prism.enableDefaultBootArtboard(false);
      }
      if (gamepad1.dpadRightWasPressed()) {
        prism.enableDefaultBootArtboard(true);
      }

      telemetry.addLine("Press A to recall Artboard #0");
      telemetry.addLine("Press B to recall Artboard #1");
      telemetry.addLine("Press Y to recall Artboard #2");
      telemetry.addLine("Press X to recall Artboard #3");
      telemetry.addLine("");
      telemetry.addLine(
          "By default the Prism will wait for an I²C signal to "
              + " start showing an animation.The prism can enable a Default Boot animation"
              + " which will play the animation stored at Artboard #0 automatically on power up.");
      telemetry.addLine(
          "Press D-Pad Right to enable the default boot animation."
              + " Press D-Pad Left to disable it.");
      telemetry.update();
      sleep(20);
    }
  }
}
