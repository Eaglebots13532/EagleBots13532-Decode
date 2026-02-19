package org.firstinspires.ftc.teamcode.drivers;

public class GameManager {
    private final AprilDriver april;
    private final GameDriver decode;
    private final ChuteDriver chute;

    public GameManager(AprilDriver april, GameDriver decode, ChuteDriver chute) {
        this.april = april;
        this.decode = decode;
        this.chute = chute;
    }

    /*

      read april tag red or blue range and angle offset.
      fly wheel velocity set by formula passing range value
      chute  angle set set by formula passing range value

      if position in field grid is known and robot outside
      april tag range set velocity and angle from field position

     */

    public AprilDriver getApril() { return this.april;  }
    public GameDriver   getGame() { return this.decode; }
    public ChuteDriver getChute() { return this.chute;  }


} //end game manager
