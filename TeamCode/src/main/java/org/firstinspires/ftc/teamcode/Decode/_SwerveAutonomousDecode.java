// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Decode;

// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

import static org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver.Artboard;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.io.File;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;

@Autonomous(name = "AutonomousDecode", group = "Decode", preselectTeleOp = "TeleOpDecode")
public class _SwerveAutonomousDecode extends LinearOpMode {
  // initialize color prism display
  GoBildaPrismDriver prism = hardwareMap.get(GoBildaPrismDriver.class, "prism");
  // get classes required
  // Create a Robot object DC_Swerve to be used to access drive hardware.
  // Prefix any hardware functions with "robot." to access this class.
  DC_Swerve_Drive robot = new DC_Swerve_Drive(this);
  // yep do the same
  DC_Intake_Launch decode = new DC_Intake_Launch(this);

  // DC_BallSensor present = new DC_BallSensor(this);
  private ElapsedTime timeOut = new ElapsedTime();

  public Telemetry telemetry;

  @Override
  public void runOpMode() {

    robot.init();
    decode.InitIL();
    // present.SensorInit();
    // tag.side = 0; // this assures tag is read and wrote

    // Wait for the DS start button to be touched.
    telemetry.addLine("Ready");
    telemetry.update();
    waitForStart();
    if (opModeIsActive()) {
      try {
        // Read Red or Blue tag and store for TeleOp
        String filename = "DecodeMatch.txt";
        // AppUtil.getSettingsFile() places the file in the FIRST/settings folder
        File file = AppUtil.getInstance().getSettingsFile(filename);
        // Get Match side to write to file
        // String match = String.valueOf(tag.side); // side passed from reading tag
        // --- Write the file ---
        telemetry.addData("Status", "Write Match to: " + filename);
        // Use the static method from ReadWriteFile to write the string data
        // ReadWriteFile.writeFile(file, match);
      } catch (Exception e) {
        telemetry.addData("Read Error", e.getMessage());
        telemetry.update();
      }
      // run autonomous actions
      timeOut.reset();
      // change spin-up velocity check range set velocity
      while (opModeIsActive() && decode.spinUp(4500.0) && timeOut.seconds() < 3.0) {
        idle();
      }
      prism.loadAnimationsFromArtboard(Artboard.ARTBOARD_0);
      Decode(); // launch first ball
      Decode(); // launch second ball
      Decode(); // launch third ball
      decode.spinOff(); // raise for next
      // rotate and drive to match side

    } // end if OpMode sequence
  } // end runOpMode

  public void Decode() {
    /* Launch requested
    1. check if ball is present
    2. There, close gate
    3. stop intake
    4. set chute they drivers have 1 sec to adjust manually
    5. spin up
    6. wait for launch command

    if (present.Present()) {
      decode.closeGate();
      decode.IntakeStop();
      decode.chute60(); // default long range 60 deg
      // wait for chute movement or decision for control chute
      sleep(1000);
    } // check for ball
    sleep(1000);

    1. launch ball
    2. open gate
    3. start intake
    4. wait for ball to clear
    5. raise arm
    6. spin off
     */
    decode.IntakeStop();
    decode.chute60(); // default long range 60 deg
    // wait for chute movement or decision for control chute
    sleep(1000);
    decode.armPosition(1); // lower to launch
    prism.loadAnimationsFromArtboard(Artboard.ARTBOARD_1);
    // check for ball present or wait 2 sec
    // decode.openGate(); // ball away ready for next
    decode.Intake(); // start intake
    sleep(1000); // wait for ball to clear or check for ball
    decode.armPosition(0);
    // decode.spinOff(); // raise for next
  } // decode
} // end autonomous
