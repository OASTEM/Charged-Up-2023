// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class CANIDS {
    public static final int DRIVETRAIN_FRONT_RIGHT = 8; // 8 //practice bot: 2
    public static final int DRIVETRAIN_BACK_RIGHT = 9; // 9 //practice bot: 3
    public static final int DRIVETRAIN_FRONT_LEFT = 7; // 7 //practice bot: 0
    public static final int DRIVETRAIN_BACK_LEFT = 6; // 6 //practice bot: 1
    public static final int ARM_ID = 11;
    public static final int SIDEARM_ID = 1;
    public static final int LEFTGRABBER_ID = 5;
    public static final int RIGHTGRABBER_ID = 4;
    public static final int OPENCLOSE_ID = 3;
    

  }

  public final static class DriveTrain {
    public static final double SLOW_MODE_TURN = 0.33;
    public static final double SLOW_MODE = 0.3;
    public static final double SUPER_SLOW_MODE = 0.1;
    public static final double REGULAR_MODE = 0.7; // 0.6
    public static final double REGULAR_MODE_TURN = 0.4; // 0.3
    public static final double MOTOR_TO_WHEEL_REVOLUTION = 10.71;
    public static final int SENSOR_UNITS_PER_ROTATION = 2048;
    public static final double TURN_CONSTANT = 240;
    public static final int DRIVE_WHEEL_DIAMETER_INCHES = 6;
    // public static final PID PID = new PID(0.1, 0.01, 0.05, 0.05, 0);
    public static final int CRUISE_VELOCITY = 7000; // max is around 21500
    public static final int ACCELERATION = 3500;
    public static final int ERROR_THRESHOLD = 500;
    public static final double OPEN_LOOP_RAMP = 1;
    public static final double CLOSED_LOOP_RAMP = .5;
    public static final double ANGLE_TOLERANCE = .5;
    public static final boolean DEBUG = false;
    public static final double TRACK_WIDTH = 20;
    public static final PID PID = new PID(0.011, 0, 0.0001, 0, 0); // 0.01015
    public static final double kP = 0.1;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double TOL = 500;
    public static final HashMap<String, Command> AUTO_EVENT_MAP = new HashMap<>();
  }

  public final static class Arm {
    public static final PID upPID = new PID(0.017, 0.0000005, 0, 0.00005, 0); // TODO fix these pid values
    public static final PID downPID = new PID(0.0102, 0.000000, 0.00, 0, 1);
    public static final PID sidePID = new PID(0.02,0.00000005,0,0, 0);//0.02
    // public static final PID autoSidePID = new PID(0.015, 0.00000001, 0, 0,1);
    public static final double ARM_SCORING_POSITION = 55000; 
    public static final double ARM_SCORING_POSITION_CUBE = 85000;
    public static final double PIVOT_START = -56; 
    public static final double ARM_LIMIT_BOTTOM = 70;
    public static final double ARM_START_POSITION = 164000;  //1600000
    public static final double PIVOT_START_POSITION = -200; //
    public static final double PIVOT_OPEN_LOOP_RATE = 0.75;// .5 //0.75
    public static final double PIVOT_CLOSED_LOOP_RATE = 0.75; // 0.75
    public static final double ARM_OPEN_LOOP_RATE = 1; // .25
    public static final double ARM_CLOSED_LOOP_RATE = 1;
    public static final double CUBE_AUTO_PIVOT = -68;
    public static final double ARM_TOL = 3500;
    public static final double PIVOT_TOL = 4; //1
    public static final double PIVOT_THRESH = 1;
    public static final double FEEDER_HEIGHT = 70000;
    public final static class SoftStop {
      public static final float ARM_UP = (float) 3000;
      public static final float ARM_DOWN = 175000;
      public static final float ARM_LEFT = -52;
      public static final float ARM_RIGHT = -200;
      public static final float PIVOT_MID_LEFT = -45;
      public static final float PIVOT_MID_RIGHT = -60;
    }
  }

  public final static class openCloseMotor {
    public static final PID openClosePID = new PID(0.04, 0.0000, 0, 0.00001); // 0.06
    public static final double conePosition = -25;
    public static final double cubePosition = -11;
    public static final double openPosition = 0;
  }
}
