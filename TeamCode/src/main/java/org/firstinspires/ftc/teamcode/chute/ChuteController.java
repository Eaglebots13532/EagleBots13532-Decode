package org.firstinspires.ftc.teamcode.chute;

/**
 * Simple chute controller with homing and position control.
 */
public class ChuteController {
    private final MockMotor motor;
    private final MockPotentiometer pot;
    private final double maxExtension;
    private final double wrapAmount;
    
    private boolean homed = false;
    private boolean homing = false;
    private double homeVoltage = 0.0;
    private double unwrappedVoltage = 0.0;
    private double currentPosition = 0.0;
    private double lastVoltage = 0.0;
    private double targetPosition = 0.0;
    private boolean moving = false;
    private boolean needsToMove = false;
    
    private int stableCount = 0;
    private static final int STABLE_REQUIRED = 5;
    private static final double TOLERANCE = 0.15; // Generous tolerance
    
    /**
     * Create a new chute controller.
     * @param motor Motor that drives the chute
     * @param pot Potentiometer that measures position
     * @param maxExtension Maximum extension from home in radians
     */
    public ChuteController(MockMotor motor, MockPotentiometer pot, double maxExtension) {
        this.motor = motor;
        this.pot = pot;
        this.maxExtension = maxExtension;
        this.wrapAmount = pot.getWrapAmount();
    }

    public double potVoltage(){
        return pot.getVoltage();
    }
    
    /**
     * Start homing sequence - drives down until pot stops changing, then captures reference voltage.
     */
    public void home() {
        motor.setPower(-0.5);
        homed = false;
        homing = true;
        stableCount = 0;
        lastVoltage = pot.getVoltage();
    }
    
    /**
     * Move to target position. Automatically homes first if not already homed.
     * @param target Target position in radians from home
     */
    public void moveToPosition(double target) {
        target = Math.max(0, Math.min(maxExtension, target));
        targetPosition = target;
        
        if (!homed) {
            needsToMove = true;
            if (!homing) {
                home();
            }
            return;
        }
        
        moving = true;
        needsToMove = false;
        
        if (target > currentPosition) {
            motor.setPower(0.7);
        } else {
            motor.setPower(-0.7);
        }
    }
    
    /**
     * Update controller state - call this every loop iteration.
     * @param dt Time since last update in seconds
     */
    public void update(double dt) {
        double oldMotorPos = motor.getPosition();
        motor.update(dt);
        double newMotorPos = motor.getPosition();
        double actualMovement = newMotorPos - oldMotorPos;
        
        pot.updatePosition(actualMovement);
        
        double voltage = pot.getVoltage();
        
        if (homing) {
            updateHoming(voltage);
        } else if (moving) {
            updateMoving(voltage);
        }
        
        lastVoltage = voltage;
    }
    
    /**
     * Update homing state - waits for stable pot readings then captures home voltage.
     */
    private void updateHoming(double voltage) {
        if (Math.abs(voltage - lastVoltage) < 0.01) {
            stableCount++;
            if (stableCount >= STABLE_REQUIRED) {
                motor.setPower(0);
                homeVoltage = voltage;
                unwrappedVoltage = voltage;
                currentPosition = 0.0;
                homed = true;
                homing = false;
                stableCount = 0;
                
                if (needsToMove) {
                    needsToMove = false;
                    moving = true;
                    if (targetPosition > 0) {
                        motor.setPower(0.7);
                    }
                }
            }
        } else {
            stableCount = 0;
        }
    }
    
    /**
     * Update moving state - tracks wraps, calculates position, stops at target.
     */
    private void updateMoving(double voltage) {
        double delta = voltage - lastVoltage;
        
        // Detect wraps and accumulate to unwrapped voltage
        // Threshold is half the wrap amount
        if (Math.abs(delta) > wrapAmount / 2) {
            if (delta < 0) {
                // Forward wrap: voltage dropped, add back wrap amount
                unwrappedVoltage += (delta + wrapAmount);
            } else {
                // Backward wrap: voltage jumped, subtract wrap amount
                unwrappedVoltage += (delta - wrapAmount);
            }
        } else {
            // Normal movement
            unwrappedVoltage += delta;
        }
        
        // Position is simply unwrapped voltage minus home
        currentPosition = unwrappedVoltage - homeVoltage;
        if (currentPosition < 0) currentPosition = 0;
        
        // Check if at target
        if (Math.abs(currentPosition - targetPosition) < TOLERANCE) {
            motor.setPower(0);
            moving = false;
        }
        
        // Safety limit
        if (currentPosition >= maxExtension) {
            motor.setPower(0);
            moving = false;
        }
    }
    
    /**
     * Check if chute has been homed.
     * @return True if homing is complete
     */
    public boolean isHomed() {
        return homed;
    }
    
    /**
     * Check if chute is currently moving to target.
     * @return True if moving
     */
    public boolean isMoving() {
        return moving;
    }
    
    /**
     * Get current position of chute.
     * @return Position in radians from home
     */
    public double getPosition() {
        return currentPosition;
    }
    
    /**
     * Get the home voltage reference captured during homing.
     * @return Home voltage in range [0, wrapAmount]
     */
    public double getHomeVoltage() {
        return homeVoltage;
    }
    
    /**
     * Get the pot wraparound amount.
     * @return Wrap amount in radians
     */
    public double getWrapAmount() {
        return wrapAmount;
    }
    
    /**
     * Emergency stop - immediately stops motor and cancels movement.
     */
    public void stop() {
        motor.setPower(0);
        moving = false;
    }
}