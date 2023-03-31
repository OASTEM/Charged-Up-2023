package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.music.Orchestra;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;

public class DriveTrain extends SubsystemBase {
  private boolean slowModeOn;
  private TalonFX frontR = new TalonFX(Constants.CANIDS.DRIVETRAIN_FRONT_RIGHT);
  private TalonFX frontL = new TalonFX(Constants.CANIDS.DRIVETRAIN_FRONT_LEFT);
  private TalonFX backR = new TalonFX(Constants.CANIDS.DRIVETRAIN_BACK_RIGHT);
  private TalonFX backL = new TalonFX(Constants.CANIDS.DRIVETRAIN_BACK_LEFT);

  Orchestra orchestraFrontR;
  Orchestra orchestraFrontL;
  Orchestra orchestraBackR;
  Orchestra orchestraBackL;

  private final AHRS navX = new AHRS(SPI.Port.kMXP, (byte) 50);
  Pose2d pose = new Pose2d();
  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(
      Units.inchesToMeters(Constants.DriveTrain.TRACK_WIDTH));
  DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading(), getLeftEncoderCount(),
      getRightEncoderCount(), pose);

  SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(0.5, 0.5);

  PIDController leftPIDController = new PIDController(Constants.DriveTrain.kP, Constants.DriveTrain.kI,
      Constants.DriveTrain.kD);
  PIDController rightPIDController = new PIDController(Constants.DriveTrain.kP, Constants.DriveTrain.kI,
      Constants.DriveTrain.kD);

  public DriveTrain() {
    slowModeOn = true;

    orchestraFrontR = new Orchestra();
    orchestraFrontL = new Orchestra();
    orchestraBackR = new Orchestra();
    orchestraBackL = new Orchestra();

    frontL.configFactoryDefault();
    frontR.configFactoryDefault();
    backL.configFactoryDefault();
    backR.configFactoryDefault();


    frontR.setNeutralMode(NeutralMode.Brake);
    frontL.setNeutralMode(NeutralMode.Brake);


    frontL.setInverted(true);
    backL.setInverted(true);

    frontR.setInverted(false);
    backR.setInverted(false);

    // frontL.configClosedloopRamp(Constants.DriveTrain.CLOSED_LOOP_RAMP);
    // frontR.configClosedloopRamp(Constants.DriveTrain.CLOSED_LOOP_RAMP);


    frontL.configOpenloopRamp(Constants.DriveTrain.OPEN_LOOP_RAMP);
    frontR.configOpenloopRamp(Constants.DriveTrain.OPEN_LOOP_RAMP);

    // Current Limit comment start
    // frontL.configPeakOutputForward(1);
    // frontL.configPeakOutputReverse(-1);
    // frontR.configPeakOutputForward(1);
    // frontR.configPeakOutputReverse(-1);

    // SupplyCurrentLimitConfiguration currentLimit = new
    // SupplyCurrentLimitConfiguration();
    // currentLimit.currentLimit = 40;
    // currentLimit.enable = true;
    // currentLimit.triggerThresholdCurrent = 40;
    // currentLimit.triggerThresholdTime = 2;
    // frontL.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40,
    // 40, 2), 0);
    // frontL.configSupplyCurrentLimit(currentLimit, 0);
    // frontR.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40,
    // 40, 2), 0);
    // frontR.configSupplyCurrentLimit(currentLimit, 0);

    // frontL.configMotionCruiseVelocity(Constants.DriveTrain.CRUISE_VELOCITY);
    // frontL.configMotionAcceleration(Constants.DriveTrain.ACCELERATION);
    // frontR.configMotionCruiseVelocity(Constants.DriveTrain.CRUISE_VELOCITY);
    // frontR.configMotionAcceleration(Constants.DriveTrain.ACCELERATION);

    // backL.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40,
    // 40, 2), 0);
    // backL.configSupplyCurrentLimit(currentLimit, 0);
    // backR.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40,
    // 40, 2), 0);
    // backR.configSupplyCurrentLimit(currentLimit, 0);

    // Current Limit comment done

    // backL.configMotionCruiseVelocity(Constants.DriveTrain.CRUISE_VELOCITY);
    // backL.configMotionAcceleration(Constants.DriveTrain.ACCELERATION);
    // backR.configMotionCruiseVelocity(Constants.DriveTrain.CRUISE_VELOCITY);
    // backR.configMotionAcceleration(Constants.DriveTrain.ACCELERATION);
    // this.setPID(Constants.DriveTrain.PID);
    backL.follow(frontL);
    backR.follow(frontR);
    resetEncoders();

  }

  public void arcadeDrive(double x, double y) {
    SmartDashboard.putNumber("X number", x);
    SmartDashboard.putNumber("y number", y);
    frontL.set(ControlMode.PercentOutput, y - x);
    frontR.set(ControlMode.PercentOutput, y + x);
  }

  public void setLeftPID(int slot_id, double p, double i, double d){
    frontL.config_kP(slot_id, p);
    frontL.config_kI(slot_id, i);
    frontL.config_kD(slot_id, d);
  }

  public void setRightPID(int slot_id, double p, double i, double d){
    frontR.config_kP(slot_id, p);
    frontR.config_kI(slot_id, i);
    frontR.config_kD(slot_id, d);
  }

  public boolean getSlowMode() {
    return slowModeOn;
  }

  public void toggleSlowMode() {
    slowModeOn = !slowModeOn;
  }

  public void setSlowMode(boolean slowMode) {
    slowModeOn = slowMode;
  }

  public void tankDrive(double left, double right) {
    frontL.set(ControlMode.PercentOutput, left);
    frontR.set(ControlMode.PercentOutput, right);
  }

  public void setLeftSpeed(double speed) {
    frontL.set(ControlMode.PercentOutput, speed);
  }

  public void setRightSpeed(double speed) {
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
  // backL.set(ControlMode.PercentOutput, 0.6);
  // }
  public double getNativeUnitsFromInches(double inches) {
    return inches * Constants.DriveTrain.MOTOR_TO_WHEEL_REVOLUTION
        / (Math.PI * Constants.DriveTrain.DRIVE_WHEEL_DIAMETER_INCHES)
        * Constants.DriveTrain.SENSOR_UNITS_PER_ROTATION;
  }

  public double getInchesFromNativeUnits(double native_units) {
    return native_units / Constants.DriveTrain.MOTOR_TO_WHEEL_REVOLUTION
        * (Math.PI * Constants.DriveTrain.DRIVE_WHEEL_DIAMETER_INCHES)
        / Constants.DriveTrain.SENSOR_UNITS_PER_ROTATION;
  }

  public double getNativeUnitsFromAngle(double degrees) {
    return degrees * Constants.DriveTrain.TURN_CONSTANT;

  }

  public void resetEncoders() {
    frontL.getSensorCollection().setIntegratedSensorPosition(0, 0);
    frontR.getSensorCollection().setIntegratedSensorPosition(0, 0);
    backL.getSensorCollection().setIntegratedSensorPosition(0, 0);
    backR.getSensorCollection().setIntegratedSensorPosition(0, 0);
    // frontL.getSensorCollection().setQuadraturePosition(0, 0);
    // frontR.getSensorCollection().setQuadraturePosition(0, 0);
  }

  public void printEncoders() {
    System.out.println("Left: " + getLeftEncoderCount());
    System.out.println("Right: " + getRightEncoderCount());
  }

  // public void printInches() {
  // System.out.println("Left Inches: " +
  // getInchesFromNativeUnits(getLeftEncoderCount()));
  // System.out.println("Right Inches: " +
  // getInchesFromNativeUnits(getRightEncoderCount()));
  // }

  // public void setAngle(double degrees) {
  // System.out.println("LEFT WAS" + this.getLeftEncoderCount());
  // System.out.println("ADDDING LEFT TO " + (degrees *
  // Constants.DriveTrain.TURN_CONSTANT));
  // System.out.println("SETTING TO " + (this.getLeftEncoderCount() + (degrees *
  // Constants.DriveTrain.TURN_CONSTANT)));
  // frontL.set(ControlMode.MotionMagic, this.getLeftEncoderCount() + (degrees *
  // Constants.DriveTrain.TURN_CONSTANT));
  // frontR.set(ControlMode.MotionMagic, this.getRightEncoderCount() - (degrees *
  // Constants.DriveTrain.TURN_CONSTANT));
  // }

  public void setPosition(double pos) {
    frontL.set(ControlMode.MotionMagic, pos);
    frontR.set(ControlMode.MotionMagic, pos);
  }

  // public void setPID(PID pid) {
  // frontL.config_kP(pid.s, pid.p);
  // frontL.config_kI(pid.s, pid.i);
  // frontL.config_kD(pid.s, pid.d);
  // frontL.config_kF(pid.s, pid.f);

  // frontR.config_kP(pid.s, pid.p);
  // frontR.config_kI(pid.s, pid.i);
  // frontR.config_kD(pid.s, pid.d);
  // frontR.config_kF(pid.s, pid.f);
  // }

  public void loadMusic() {
    orchestraFrontR.loadMusic("fight song.chrp");
    orchestraFrontL.loadMusic("fight song.chrp");
    orchestraBackR.loadMusic("fight song.chrp");
    orchestraBackL.loadMusic("fight song.chrp");
  }

  public void addInstruments() {
    orchestraFrontL.addInstrument(frontL);
    orchestraBackL.addInstrument(backL);
    orchestraFrontR.addInstrument(frontR);
    orchestraBackR.addInstrument(backR);
  }

  public void playMusic() {
    orchestraFrontL.play();
    orchestraBackR.play();
    orchestraBackL.play();
    orchestraFrontR.play();
  }

  public boolean playFrontL() {
    orchestraFrontL.play();
    return true;
  }

  public boolean playFrontR() {
    orchestraFrontR.play();
    return true;
  }

  public boolean playBackL() {
    orchestraBackL.play();
    return true;
  }

  public boolean playBackR() {
    orchestraBackR.play();
    return true;
  }

  public void stopMusic() {
    orchestraFrontL.stop();
    orchestraBackR.stop();
    orchestraFrontR.stop();
    orchestraBackL.stop();
  }

  public double getZAngle() {
    return navX.getAngle();
  }

  public double getXAngle() {
    return navX.getRoll();
  }

  public double getYAngle() {
    return navX.getPitch();
  }

  public Pose2d getPose() {
    System.out.println("GOT SPEED");
    return pose;
  }

  public void setOutput(double left, double right) {
    System.out.println(left + "  " + right + " OUTPUT");
    frontL.set(ControlMode.PercentOutput, left);
    frontR.set(ControlMode.PercentOutput, right);
  }

  public void reset() {
    navX.reset();
  }

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-navX.getAngle());
  }

  public DifferentialDriveWheelSpeeds getSpeeds() {
    System.out.println("GOT SPEED");
    return new DifferentialDriveWheelSpeeds(
        frontL.getSelectedSensorVelocity() * 7.31 * 2 * Math.PI * Units.inchesToMeters(4) / 60,
        frontR.getSelectedSensorVelocity() * 7.31 * 2 * Math.PI * Units.inchesToMeters(4) / 60);
  }

  public SimpleMotorFeedforward getFeedForward() {
    return feedForward;
  }

  public PIDController getLeftPIDController() {
    return leftPIDController;
  }

  public PIDController getRightPIDController() {
    return rightPIDController;
  }

  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }

  public void RemoveDriveTrainRampRate(){
    frontL.configOpenloopRamp(0);
    frontR.configOpenloopRamp(0);
    backL.configOpenloopRamp(0);
    backR.configOpenloopRamp(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("NavX X", getXAngle());
    SmartDashboard.putNumber("NavX Y", getYAngle());
    SmartDashboard.putNumber("NavX Z", getZAngle());
    pose = odometry.update(getHeading(), getLeftEncoderCount(), getRightEncoderCount());
    // printEncoders();
    SmartDashboard.putNumber("Right Inches" ,getInchesFromNativeUnits(getRightEncoderCount()));
    // System.out.println("Inches from native units: " + getInchesFromNativeUnits(getLeftEncoderCount()));
    SmartDashboard.putNumber("DriveTrain L Encoder", getLeftEncoderCount());
    SmartDashboard.putNumber("DriveTrain R Encoder", getRightEncoderCount());
    // SmartDashboard.putNumber("FrontL Current", frontL.getSupplyCurrent());
    // SmartDashboard.putNumber("FrontR Current", frontR.getSupplyCurrent());
  }
}
