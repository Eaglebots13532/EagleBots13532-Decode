package org.firstinspires.ftc.teamcode.Decode;

import org.firstinspires.ftc.teamcode.drivers.GameDriver;
import org.firstinspires.ftc.teamcode.drivers.odo.GoBildaPinpointDriver;

public class CarlHoodShoot {

    GameDriver motors;
    GoBildaPinpointDriver odo;

    public CarlHoodShoot(GameDriver motors, GoBildaPinpointDriver odo) {
        this.motors = motors;
        this.odo = odo;
    }


    double encCountToLaunchAngle = 35.95 / 1117.84;
    double targetHeight = 1200; //Target height for the ball to be at
    double targetDistance;
    double angleError;
    double lastAngleError;
    double hoodAngle;
    double hoodIntegral;
    double kP = 0.05;
    double kI = 0.00;
    double kD = 0.005;
    double lastTime;
    double changeTime;
    //Parameters for speed of ball equation
    final double massBall = 75; //in grams
    final double radiusBallPath = (104 / 2) / 10; //Radius of the ball during the path of acceleration in the shooter I believe, in Cm
    final double flyWheelMOI = 388.8; //Moment of inertia of the flywheel, g * cm^2
    final double flyWheelMass = 600; //Flywheel mass in grams
    boolean isHoming = false;


    public double getHoodServoPowerPID(double seekAngle, double elapsedTime) {
        double derivative;
        hoodAngle = motors.getHoodEncoderPos() * 35.95 / 1117 * 4 * 90 / 100;
        angleError = seekAngle - hoodAngle;

        changeTime = elapsedTime - lastTime;

        derivative = (angleError - lastAngleError) / changeTime;

        hoodIntegral += angleError * changeTime;
        hoodIntegral = Math.max(-0.1, Math.min(0.1, hoodIntegral)); //clamp integral to keep from exploading in value

        double out = ((kP * angleError) + (kI * hoodIntegral) + (kD * derivative));

        lastAngleError = angleError;
        lastTime = elapsedTime;

        return out;
    }

    public double getChangeTime() {
        return changeTime;
    }

    public double getHoodAngle() {
        return hoodAngle;
    }

    public double getAngleError() {
        return angleError;
    }

    /*
    public void homeHooD () {
        if (motors.getTouchSensor()) {
            motors.setHoodServoPower(-0.5);
        } else if (!motors.getTouchSensor()) {
            motors.setHoodServoPower(0);
            motors.resetRevEncoderPos();
        }
    }

     */
    double theta1;
    double theta2;

    public double getLaunchAngle(double x, double y, double velocity, boolean isTheta1) {
        double g = 9.81;

        //Sets up quadratic
        double a = (g * x * x) / (2 * velocity * velocity);
        double b = -x;
        double c = y + a;

        //Evaluates the square root part of the quadratic equation
        double discriminant = b * b - 4 * a * c;

        if (discriminant < 0) {
            return Double.NaN; // No valid shot
        }

        double sqrt = Math.sqrt(discriminant);

        // Two possible solutions
        double T1 = (-b + sqrt) / (2 * a);
        double T2 = (-b - sqrt) / (2 * a);

        // Choose the lower angle solution
        theta1 = Math.toDegrees(Math.atan(T1));
        theta2 = Math.toDegrees(Math.atan(T2));

        //Returns smallest angle
        if (isTheta1) {
            return theta1;
        } else {
            return theta2;
        }
        //return Math.min(theta1, theta2);
    }

    public double getTheta1() {
        return theta1;
    }

    public double getTheta2() {
        return theta2;
    }


    public double getBallVelFromFly(double flyVel) {
        double flyVelRadiansPerSec = flyVel * 6.28 / 60;   //Converting from RPM to Rad/s
        double ballVel;
        ballVel = (flyWheelMOI * flyVelRadiansPerSec) / (flyWheelMOI + (massBall * radiusBallPath)) * (104 / 2) / 10; //Final clause converts from rad/s to cm/s
        //Returns in Cm/S
        return ballVel;
    }

    public double getBallVelFromFinalFly(double flyVel) {
        //(Mass flywheel * flyVel)/(Mass flywheel + 2 mass ball)
        double finalVel = ((flyWheelMass * (flyVel / 60)) / (flyWheelMass + (2 * massBall))) * (104 / 2) / 1000; //Converts flyVel from Rpm to Rps, then multiplies by radius
        return finalVel;
    }

    public double getAccurateBallVelFromFinalFly(double flyVel) {
        //(Mass flywheel * flyVel)/(Mass flywheel + 2 mass ball)
        double finalVel = ((flyWheelMOI * (flyVel / 60)) / (flyWheelMOI + (massBall * 5.2 * 5.2)) * 5.2) / 100; //Converts flyVel from Rpm to Rps, then multiplies by radius
        return finalVel;
    }

    public double getFinalFlyVel(double flyVel) {
        //(Mass flywheel * flyVel)/(Mass flywheel + 2 mass ball)
        double finalVel = ((flyWheelMass * (flyVel)) / (flyWheelMass + (2 * massBall))); //Converts flyVel to final velocity
        return finalVel;
    }

    public void hoodHome() {
        isHoming = true;
        if (motors.getTouchSensor() && isHoming) {
            motors.setHoodServoPower(-0.5);
        } else if (!motors.getTouchSensor() && isHoming) {
            motors.setHoodServoPower(0);
            isHoming = false;
            motors.setHoodServoPower(0);
        }
    }
    public boolean hasHomed () {
        return isHoming;
    }
}
