// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Prism;

import static org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver.LayerHeight;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.concurrent.TimeUnit;

/*
 * This example file shows how to create a couple of different Animations on the Prism, and save
 * them to an Artboard on the device.
 * The example file called "GoBildaPrismConfigurator" is designed to create a user-interface
 * through your driver's station. This is a great way to create Artboards from Animations, but
 * doesn't let you control quite everything about the available Animations. If you'd like to
 * do something more complex, or would just prefer to create your Artboard in Java, this
 * program shows you how.
 * The example file called "GoBildaPrismArtboardExample" shows how to recall different Artboards
 * that you have already saved to the device. That file shows the code you should consider adding
 * to your OpMode if you would like to dynamically change the Artboard shown by your LEDs during
 * the match.
 *
 * Core to understanding how to use this product is knowing these three terms:
 * Animations: (Like RAINBOW or BLINK) - These have properties you can configure, like their color.
 * They can have unique start and end points!
 * Layers: There are 10 layers, each of which can store an animation. These are hierarchical.
 * So an Animation on layer 5 will cover an animation on layer 2 if they overlap.
 * You can use start and end points to have layers overlap to create new patterns! Or show multiple
 * animations at once on different LEDs.
 * Artboards: An Artboard is a set of 10 layers which is stored on the Prism.
 * you can have up to 8 unique Artboards. Artboards are easy and computationally fast to switch between.
 */

@Disabled
@TeleOp(name = "Prism Animations Example", group = "Linear OpMode")

public class GoBildaPrismExample extends LinearOpMode {

  GoBildaPrismDriver prism;

  PrismAnimations.Solid solid = new PrismAnimations.Solid(Color.BLUE);
  PrismAnimations.RainbowSnakes rainbowSnakes = new PrismAnimations.RainbowSnakes();

  @Override
  public void runOpMode() {
    /*
     * Initialize the hardware variables. Note that the strings used here must correspond
     * to the names assigned during the robot configuration step on the driver's station.
     */
    prism = hardwareMap.get(GoBildaPrismDriver.class, "prism");

    /*
     * Here you can customize the specifics of different animations. Each animation has it's
     * own set of parameters that you can customize to create something unique! Each animation
     * has carefully selected default parameters. So you do not need to set each parameter
     * for every animation!
     */
    solid.setBrightness(50);
    solid.setStartIndex(0);
    solid.setStopIndex(12);

    rainbowSnakes.setNumberOfSnakes(2);
    rainbowSnakes.setSnakeLength(3);
    rainbowSnakes.setSpacingBetween(6);
    rainbowSnakes.setSpeed(0.5f);

    telemetry.addData("Device ID: ", prism.getDeviceID());
    telemetry.addData("Firmware Version: ", prism.getFirmwareVersionString());
    telemetry.addData("Hardware Version: ", prism.getHardwareVersionString());
    telemetry.addData("Power Cycle Count: ", prism.getPowerCycleCount());
    telemetry.update();

    // Wait for the game to start (driver presses START)
    waitForStart();
    resetRuntime();

    // run until the end of the match (driver presses STOP)
    while (opModeIsActive()) {

      if (gamepad1.aWasPressed()) {
        /*
         * Here we insert and update the animation to the Prism, this by default does not
         * save it to an Artboard, it just starts the Animation playing. If you have
         * already inserted an animation at a layer height, you can instead call
         * .updateAnimationFromIndex(LayerHeight.LAYER_0) to update an animation at a
         * specific layer height without overwriting it completely.
         */
        prism.insertAndUpdateAnimation(LayerHeight.LAYER_0, solid);
        prism.insertAndUpdateAnimation(LayerHeight.LAYER_1, rainbowSnakes);
      }

      if (gamepad1.xWasPressed()) {
        /*
         * Clearing the animation doesn't erase any saved Artboards, but it removes all the
         * currently displayed animations.
         */
        prism.clearAllAnimations();
      }

      if (gamepad1.dpadDownWasPressed()) {
        /*
         * Here we save the animation we are currently displaying to Artboard 0.
         */
        prism.saveCurrentAnimationsToArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_0);
      }

      telemetry.addLine("Press A to insert and update the created animations.");
      telemetry.addLine("Press X to clear current animations.");
      telemetry.addLine("Press D-Pad Down to save current animations to Artboard #0");
      telemetry.addLine();
      telemetry.addData("Run Time (Hours): ", prism.getRunTime(TimeUnit.HOURS));
      telemetry.addData("Run Time (Minutes): ", prism.getRunTime(TimeUnit.MINUTES));
      telemetry.addData("Number of LEDS: ", prism.getNumberOfLEDs());
      telemetry.addData("Current FPS: ", prism.getCurrentFPS());
      telemetry.update();
      sleep(50);
    }
  }
}
