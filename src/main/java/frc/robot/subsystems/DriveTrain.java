package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.music.Orchestra;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
import frc.robot.utils.PID;

public class DriveTrain extends SubsystemBase {
  private boolean slowModeOn;
  private boolean climbing;
  private TalonSRX frontR = new TalonSRX(Constants.CANIDS.DRIVETRAIN_FRONT_RIGHT);
  private TalonSRX frontL = new TalonSRX(Constants.CANIDS.DRIVETRAIN_FRONT_LEFT);
  private TalonSRX backR = new TalonSRX(Constants.CANIDS.DRIVETRAIN_BACK_RIGHT);
  private TalonSRX backL = new TalonSRX(Constants.CANIDS.DRIVETRAIN_BACK_LEFT);

  Orchestra orchestra1;
  Orchestra orchestra2;
  Orchestra orchestra3;
  Orchestra orchestra4;

  private final AHRS navX = new AHRS(Port.kMXP, (byte) 50);
  Pose2d pose = new Pose2d();
  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(Constants.DriveTrain.TRACK_WIDTH));
  DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading(), getLeftEncoderCount(), getRightEncoderCount(), pose);

  SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(0.5, 0.5);

  PIDController leftPIDController = new PIDController(Constants.DriveTrain.kP, Constants.DriveTrain.kI, Constants.DriveTrain.kD);
  PIDController rightPIDController = new PIDController(Constants.DriveTrain.kP, Constants.DriveTrain.kI, Constants.DriveTrain.kD);


  public DriveTrain() {

    // frontR = new TalonSRX(Constants.CANIDS.DRIVETRAIN_FRONT_RIGHT);
    // frontL = new TalonSRX(Constants.CANIDS.DRIVETRAIN_FRONT_LEFT);
    // backR = new TalonSRX(Constants.CANIDS.DRIVETRAIN_BACK_RIGHT);
    // backL = new TalonSRX(Constants.CANIDS.DRIVETRAIN_BACK_LEFT);
    slowModeOn = true;
    climbing = false;


    orchestra1 = new Orchestra();
    orchestra2 = new Orchestra();
    orchestra3 = new Orchestra();
    orchestra4 = new Orchestra();

    frontR.setNeutralMode(NeutralMode.Brake);
    frontL.setNeutralMode(NeutralMode.Brake);
    backR.setNeutralMode(NeutralMode.Brake);
    backL.setNeutralMode(NeutralMode.Brake);


    backL.follow(frontL);
    backR.follow(frontR);
    frontL.setInverted(false);
    backL.setInverted(false);
    frontR.setInverted(true);
    backR.setInverted(true);

    // frontL.configClosedloopRamp(Constants.DriveTrain.CLOSED_LOOP_RAMP);
    // frontR.configClosedloopRamp(Constants.DriveTrain.CLOSED_LOOP_RAMP);
    // backL.configClosedloopRamp(Constants.DriveTrain.CLOSED_LOOP_RAMP);
    // backR.configClosedloopRamp(Constants.DriveTrain.CLOSED_LOOP_RAMP);

    // frontL.configOpenloopRamp(Constants.DriveTrain.OPEN_LOOP_RAMP);
    // frontR.configOpenloopRamp(Constants.DriveTrain.OPEN_LOOP_RAMP);
    // backL.configOpenloopRamp(Constants.DriveTrain.OPEN_LOOP_RAMP);
    // backR.configOpenloopRamp(Constants.DriveTrain.OPEN_LOOP_RAMP);

    frontL.configPeakOutputForward(1);
    frontL.configPeakOutputReverse(-1);
    frontR.configPeakOutputForward(1);
    frontR.configPeakOutputReverse(-1);

    SupplyCurrentLimitConfiguration currentLimit = new SupplyCurrentLimitConfiguration();
    currentLimit.currentLimit = 40;
    currentLimit.enable = true;
    currentLimit.triggerThresholdCurrent = 40;
    currentLimit.triggerThresholdTime = 2;
    frontL.configSupplyCurrentLimit(currentLimit, 0);
    frontR.configSupplyCurrentLimit(currentLimit, 0);


    // frontL.configMotionCruiseVelocity(Constants.DriveTrain.CRUISE_VELOCITY);
    // frontL.configMotionAcceleration(Constants.DriveTrain.ACCELERATION);
    // frontR.configMotionCruiseVelocity(Constants.DriveTrain.CRUISE_VELOCITY);
    // frontR.configMotionAcceleration(Constants.DriveTrain.ACCELERATION);
    // this.setPID(Constants.DriveTrain.PID);

  }



  public void arcadeDrive(double x, double y) {
    frontL.set(ControlMode.PercentOutput, y - x);
    frontR.set(ControlMode.PercentOutput, y + x);
  }

  public boolean getSlowMode() {
    return slowModeOn;
  }

  public boolean climbing(){
    return climbing;
  }

  public void toggleSlowMode() {
    slowModeOn = ! slowModeOn;
  }

  public void falseSlowMode(){
    slowModeOn = false;
  }

  public void trueSlowMode(){
    slowModeOn = true;
  }

  public void setSlowMode(boolean slowMode) {
    slowModeOn = slowMode;
  }

  public void tankDrive(double left, double right) {
    frontL.set(ControlMode.PercentOutput, left);
    frontR.set(ControlMode.PercentOutput, right);
  }

  public void setLeftSpeed(double speed){
    frontL.set(ControlMode.PercentOutput, speed);
  }

  public void setRightSpeed(double speed){
    frontR.set(ControlMode.PercentOutput, speed);
  }

  public void stop() {
    frontL.set(ControlMode.PercentOutput, 0.0);
    frontR.set(ControlMode.PercentOutput, 0.0);
  }

  public double getLeftEncoderCount() {
    return frontL.getSelectedSensorPosition();

  }

  public double getRightEncoderCount() {
    return frontR.getSelectedSensorPosition();

  }



  // public void setBackLeftSpeed() {
  //   backL.set(ControlMode.PercentOutput, 0.6);
  // }
//   public double getNativeUnitsFromInches(double inches) {
//     return inches * Constants.DriveTrain.MOTOR_TO_WHEEL_REVOLUTION / (Math.PI * Constants.DriveTrain.DRIVE_WHEEL_DIAMETER_INCHES)
//         * Constants.DriveTrain.SENSOR_UNITS_PER_ROTATION;
//   }

//   public double getInchesFromNativeUnits(double native_units) {
//     return native_units / Constants.DriveTrain.MOTOR_TO_WHEEL_REVOLUTION * (Math.PI * Constants.DriveTrain.DRIVE_WHEEL_DIAMETER_INCHES)
//         / Constants.DriveTrain.SENSOR_UNITS_PER_ROTATION;
//   }

//   public double getNativeUnitsFromAngle(double degrees) {
//     return degrees * Constants.DriveTrain.TURN_CONSTANT;

//   }

  // public void resetEncoders() {
  //   frontL.getSensorCollection().setIntegratedSensorPosition(0, 0);
  //   frontR.getSensorCollection().setIntegratedSensorPosition(0, 0);
  // }

  public void printEncoders() {
    System.out.println("Left: " + getLeftEncoderCount());
    System.out.println("Right: " + getRightEncoderCount());
  }

//   public void printInches() {
//     System.out.println("Left Inches: " + getInchesFromNativeUnits(getLeftEncoderCount()));
//     System.out.println("Right Inches: " + getInchesFromNativeUnits(getRightEncoderCount()));
//   }

//   public void setAngle(double degrees) {
//     System.out.println("LEFT WAS" + this.getLeftEncoderCount());
//     System.out.println("ADDDING LEFT TO " + (degrees * Constants.DriveTrain.TURN_CONSTANT));
//     System.out.println("SETTING TO " + (this.getLeftEncoderCount() + (degrees * Constants.DriveTrain.TURN_CONSTANT)));
//     frontL.set(ControlMode.MotionMagic, this.getLeftEncoderCount() + (degrees * Constants.DriveTrain.TURN_CONSTANT));
//     frontR.set(ControlMode.MotionMagic, this.getRightEncoderCount() - (degrees * Constants.DriveTrain.TURN_CONSTANT));
//   }
  
  public void setPosition(double pos) {
    frontL.set(ControlMode.MotionMagic, pos);
    frontR.set(ControlMode.MotionMagic, pos);
  }

//   public void setPID(PID pid) {
//     frontL.config_kP(pid.s, pid.p);
//     frontL.config_kI(pid.s, pid.i);
//     frontL.config_kD(pid.s, pid.d);
//     frontL.config_kF(pid.s, pid.f);

//     frontR.config_kP(pid.s, pid.p);
//     frontR.config_kI(pid.s, pid.i);
//     frontR.config_kD(pid.s, pid.d);
//     frontR.config_kF(pid.s, pid.f);
//   }

  public void loadMusic(){
    orchestra1.loadMusic("fight song.chrp");
    orchestra2.loadMusic("fight song.chrp");
    orchestra3.loadMusic("fight song.chrp");
    orchestra4.loadMusic("fight song.chrp");
  }

  // public void addInstruments(){
  //   orchestra1.addInstrument(frontL);
  //   orchestra2.addInstrument(backL);
  //   orchestra3.addInstrument(frontR);
  //   orchestra4.addInstrument(backR);
  // }

  public void playMusic(){
    orchestra1.play();
    orchestra2.play();
    orchestra3.play();
    orchestra4.play();
  }

  public void stopMusic(){
    orchestra1.stop();
    orchestra2.stop();
    orchestra3.stop();
    orchestra4.stop();
  }

  public double getZAngle() {
    return navX.getAngle();
  }

  public double getXAngle(){
    return navX.getRoll();
}

  public double getYAngle(){
    return navX.getPitch();
}

  public Pose2d getPose() {
    return pose;
  }

  public void setOutput(double left, double right) {
    System.out.println(left + "  " + right);
    frontL.set(ControlMode.PercentOutput, left);
    frontR.set(ControlMode.PercentOutput, right);
  }

  public void reset() {
    navX.reset();
  }

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-navX.getAngle());
  }

  public DifferentialDriveWheelSpeeds getSpeeds(){
    return new DifferentialDriveWheelSpeeds(
      frontL.getSelectedSensorVelocity() * 7.31 * 2 * Math.PI * Units.inchesToMeters(4) / 60, 
      frontR.getSelectedSensorVelocity() * 7.31 * 2 * Math.PI * Units.inchesToMeters(4) / 60
      );
  }



  @Override
  public void periodic(){
    pose = odometry.update(getHeading(), getLeftEncoderCount(), getRightEncoderCount());
  }

  public SimpleMotorFeedforward getFeedForward(){
    return feedForward;
  }

  public PIDController getLeftPIDController(){
    return leftPIDController;
  }

  public PIDController getRightPIDController(){
    return rightPIDController;
  }
  
  public DifferentialDriveKinematics getKinematics(){
    return kinematics;
  }

}
