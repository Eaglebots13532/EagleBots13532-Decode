package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.subsystems.chute.Chute;
import org.firstinspires.ftc.teamcode.subsystems.chute.FtcMotor;
import org.firstinspires.ftc.teamcode.subsystems.chute.FtcPotentiometer;
import org.firstinspires.ftc.teamcode.subsystems.chute.ChuteController;

/**
 * Example OpMode demonstrating chute control with real hardware.
 * 
 * Hardware Config:
 * - Motor: "chute_motor" (DcMotor)
 * - Potentiometer: "chute_pot" (AnalogInput)
 */
@TeleOp(name = "Chute Control")
public class ChuteOpMode extends LinearOpMode {
    
    private Chute chute;
    
    // Pot configuration - change this if your pot has different range
    private static final double POT_WRAP_AMOUNT = 2 * Math.PI;
    
    // Position presets (in radians)
    private static final double HOME = 0.0;
    private static final double LOW = Math.PI;
    private static final double MID = 2 * Math.PI;
    private static final double HIGH = 3 * Math.PI;
    private static final double MAX = 4 * Math.PI;
    
    @Override
    public void runOpMode() {
        // Get hardware from config
        DcMotor chuteMotor = hardwareMap.get(DcMotor.class, "chute_motor");
        AnalogInput chutePot = hardwareMap.get(AnalogInput.class, "chute_pot");
        
        // Create hardware adapters
        FtcMotor motor = new FtcMotor(chuteMotor);
        FtcPotentiometer pot = new FtcPotentiometer(chutePot, POT_WRAP_AMOUNT);
        
        // Create chute controller with real hardware
        ChuteController controller = new ChuteController(motor, pot, MAX);
        chute = new Chute(controller, motor, pot);
        
        telemetry.addLine("Chute initialized");
        telemetry.addData("Pot wrap", "%.2f rad", POT_WRAP_AMOUNT);
        telemetry.addLine("Press A to home");
        telemetry.update();
        
        waitForStart();
        
        while (opModeIsActive()) {
            // Update chute (50Hz)
            chute.update(0.02);
            
            // Control with gamepad
            handleControls();
            
            // Display status
            updateTelemetry();
            
            sleep(20); // 50Hz loop
        }
    }
    
    private void handleControls() {
        // Manual homing
        if (gamepad1.a) {
            chute.home();
        }
        
        // Emergency stop
        if (gamepad1.b) {
            chute.stop();
        }
        
        // Position presets
        if (gamepad1.dpad_down) {
            chute.setTargetPosition(HOME);
        }
        if (gamepad1.dpad_left) {
            chute.setTargetPosition(LOW);
        }
        if (gamepad1.dpad_right) {
            chute.setTargetPosition(MID);
        }
        if (gamepad1.dpad_up) {
            chute.setTargetPosition(HIGH);
        }
        
        // Fine control with triggers
        if (gamepad1.left_trigger > 0.1) {
            double current = chute.getPosition();
            chute.setTargetPosition(current - 0.1);
        }
        if (gamepad1.right_trigger > 0.1) {
            double current = chute.getPosition();
            chute.setTargetPosition(current + 0.1);
        }
    }
    
    private void updateTelemetry() {
        telemetry.addLine("=== Chute Status ===");
        telemetry.addData("Homed", chute.isHomed() ? "YES" : "NO");
        telemetry.addData("Moving", chute.isMoving() ? "YES" : "NO");
        telemetry.addData("Position", "%.2f rad (%.0f°)", 
            chute.getPosition(), Math.toDegrees(chute.getPosition()));
        
        if (chute.isHomed()) {
            double percent = (chute.getPosition() / MAX) * 100;
            telemetry.addData("Extension", "%.0f%%", percent);
        }
        
        telemetry.addLine();
        telemetry.addLine("=== Controls ===");
        telemetry.addLine("A: Home | B: Stop");
        telemetry.addLine("D-Pad: Presets");
        telemetry.addLine("Triggers: Fine adjust");
        
        telemetry.update();
    }
}