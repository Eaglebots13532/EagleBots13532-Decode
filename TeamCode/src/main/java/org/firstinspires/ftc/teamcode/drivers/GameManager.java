package org.firstinspires.ftc.teamcode.drivers;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class GameManager {
    private final AprilDriver april;
    private final GameDriver decode;
    private final ChuteDriver chute;
    private final Telemetry telemetry;

    public GameManager(AprilDriver april, GameDriver decode, ChuteDriver chute, Telemetry telemetry) {
        this.april = april;
        this.decode = decode;
        this.chute = chute;
        this.telemetry = telemetry;
    }

    /*

      read april tag red or blue range and angle offset.
      fly wheel velocity set by formula passing range value
      chute  angle set set by formula passing range value

      if position in field grid is known and robot outside
      april tag range set velocity and angle from field position

     */
    public void initTags(){
        april.initAprilTag();
    }

    public void getTagData(){

        april.getAprilTag();
    }

    public double range(){
        return april.getRange();
    }

    public int getTag(){
        return april.getMetaId();
    }

    public double setFlyWheel(){
        double flyvelocity = 1850;
        getTagData();
        // in teleOp we are facing the correct april tag
        // at present there is no check for match tag
        if(getTag() == 24 || getTag() == 20){
            flyvelocity = 1.6374 * range() + 1506;
        }
        return flyvelocity;

    }

    public double HoodVolt(){
        // set value for middle of field
        double hoodRange = 4.5;
        getTagData();
        // in teleOp we are facing the correct april tag
        // at present there is no check for match tag
        if(getTag() == 24 || getTag() == 20){
            hoodRange = -.024 * range() + 2.7586;
        }
        return hoodRange;
    }

} //end game manager
