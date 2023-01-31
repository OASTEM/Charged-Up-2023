// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class CANIDS{
    public static final int DRIVETRAIN_FRONT_RIGHT = 3; //8
    public static final int DRIVETRAIN_BACK_RIGHT = 2; //9
    public static final int DRIVETRAIN_FRONT_LEFT = 1; // 7
    public static final int DRIVETRAIN_BACK_LEFT = 0; //6
    public static final int ARM_ID = 13;
    public static final int SIDEARM_ID = 5;
    public static final int LEFTGRABBER_ID = 2;
    public static final int RIGHTGRABBER_ID = 3;
    public static final int OPENCLOSE_ID = 4;
    
  }

  public final static class DriveTrain {
    public static final double SLOW_MODE_TURN = 0.3;
    public static final double SLOW_MODE = 0.2;
    public static final double SUPER_SLOW_MODE = 0.1;
    public static final double REGULAR_MODE = 0.6; //0.6
    public static final double REGULAR_MODE_TURN = 0.3; //0.3
    public static final double MOTOR_TO_WHEEL_REVOLUTION = 10.71;
    public static final int SENSOR_UNITS_PER_ROTATION = 2048;
    public static final double TURN_CONSTANT = 240;
    public static final int DRIVE_WHEEL_DIAMETER_INCHES = 6;
    //public static final PID PID = new PID(0.1, 0.01, 0.05, 0.05, 0);
    public static final int CRUISE_VELOCITY = 7000; // max is around 21500
    public static final int ACCELERATION = 3500;
    public static final int ERROR_THRESHOLD = 500;
    public static final double OPEN_LOOP_RAMP = .5;
    public static final double CLOSED_LOOP_RAMP = 0;
    public static final double ANGLE_TOLERANCE = .5;
    public static final boolean DEBUG = false;
  }

  public final static class Arm {
    public static final PID PID = new PID(0.0001, 0.00001, 0.08,0);
  }

  public final static class Grabber {
    public static final PID PID = new PID(0,0,0,0);
  }

}
