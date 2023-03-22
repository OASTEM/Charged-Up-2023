// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
import frc.robot.utils.PID;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private TalonFX arm;
  private CANSparkMax sideMotor;
  private SparkMaxPIDController sideMotorPIDController;
  private RelativeEncoder sideMotorEncoder;
//TODO: Encoder values (max and min for soft stop)
//TODO: PID values for side and arm
//TODO: Current values for calibration
//TODO: Clean up Code
  public Arm() {
    sideMotor = new CANSparkMax(Constants.CANIDS.SIDEARM_ID, MotorType.kBrushless);
    arm = new TalonFX(Constants.CANIDS.ARM_ID);

    System.out.println("CALLED ARM CONSTRUCTOR");
    resetDefault();
    sideMotor.enableSoftLimit(SoftLimitDirection.kForward, false);
    sideMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);

    sideMotorPIDController = sideMotor.getPIDController();
    sideMotorEncoder = sideMotor.getEncoder();

    sideMotorPIDController.setP(Constants.Arm.sidePID.p);
    sideMotorPIDController.setI(Constants.Arm.sidePID.i);
    sideMotorPIDController.setD(Constants.Arm.sidePID.d);

    sideMotor.setClosedLoopRampRate(Constants.Arm.PIVOT_CLOSED_LOOP_RATE);
    sideMotor.setOpenLoopRampRate(Constants.Arm.PIVOT_OPEN_LOOP_RATE);

    arm.setNeutralMode(NeutralMode.Brake);

    arm.configPeakOutputForward(1);
    arm.configPeakOutputReverse(-1);

    initPIDController(Constants.Arm.upPID);

    configRampRates();

    // sideMotor.setSmartCurrentLimit(30);
    // armMotor.setSmartCurrentLimit(40);
    // sideMotor.setSecondaryCurrentLimit(35);
    // armMotor.setSecondaryCurrentLimit(45);
    System.out.println("In Arm Constructor ************");
  }
  
  public void initPIDController( PID pid){
    arm.config_kP(pid.s, pid.p);
    arm.config_kI(pid.s, pid.i);
    arm.config_kD(pid.s, pid.d);
  }

  public void configRampRates(){
    arm.configOpenloopRamp(Constants.Arm.ARM_OPEN_LOOP_RATE);
    arm.configClosedloopRamp(Constants.Arm.ARM_CLOSED_LOOP_RATE);
  }

  public void setSpeed(double speed){
    arm.set(ControlMode.PercentOutput, speed);
    // System.out.println("***********************************setting falcon speed");
  }

  public void setPosition(double encoder){
    arm.set(ControlMode.Position, encoder);
  }

  public void resetEncoders() {
    arm.getSensorCollection().setIntegratedSensorPosition(0, 0);
  }

  public double getEncoderCount() {
    //return arm.getSensorCollection().getIntegratedSensorPosition();
    return arm.getSelectedSensorPosition();
  }

  public double getCurrent() {
    return Math.abs(arm.getStatorCurrent());
  }

  public double getVelocity(){
    return arm.getSelectedSensorVelocity();
  }


  // Side Arm
  public void setSideMotorPosition(double position) {
    sideMotorPIDController.setReference(position, CANSparkMax.ControlType.kPosition);
  }

  public void resetSideEncoders() {
    sideMotorEncoder.setPosition(0);
  }

  public void setSide(double speed) {
    sideMotor.set(speed);
  }

  public void setSideVelocity(int velocity) {
    sideMotorPIDController.setReference(velocity, CANSparkMax.ControlType.kVelocity);
  }

  public void setSidePID(PID pid) {
    sideMotorPIDController.setP(pid.p);
    sideMotorPIDController.setI(pid.i);
    sideMotorPIDController.setD(pid.d);
  }

  public double getSideEncoder() {
    return sideMotor.getEncoder().getPosition();
  }

  public double getSideCurrent() {
    return Math.abs(sideMotor.getOutputCurrent());
  }

  public void setPivotRampRate(){
    sideMotor.setClosedLoopRampRate(Constants.Arm.PIVOT_CLOSED_LOOP_RATE);
    sideMotor.setOpenLoopRampRate(Constants.Arm.PIVOT_OPEN_LOOP_RATE);
  }

  public void setSideRange() { //TODO: test this
    sideMotorPIDController.setOutputRange(-0.2, 0.2);
  }

  // Stop
  public void stop() {
    arm.set(ControlMode.PercentOutput, 0.0);
    sideMotor.stopMotor();
  }

  public void enableArmSoftLimit() {
    // arm.configForwardSoftLimitEnable(true, 0);
    // arm.configReverseSoftLimitEnable(true, 0);
    sideMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    sideMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    System.out.println("Enableing soft limit ******************************************");
  }

  public void disableArmSoftLimit() {
    // arm.configForwardSoftLimitEnable(false, 0);
    // arm.configReverseSoftLimitEnable(false, 0);
    sideMotor.enableSoftLimit(SoftLimitDirection.kForward, false);
    sideMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
  }

  public void setArmSoftLimit() {
    arm.configForwardSoftLimitThreshold(Constants.Arm.SoftStop.ARM_UP, 0);
    arm.configReverseSoftLimitThreshold(Constants.Arm.SoftStop.ARM_DOWN, 0);
    sideMotor.setSoftLimit(SoftLimitDirection.kForward, Constants.Arm.SoftStop.ARM_LEFT); // kForward
    sideMotor.setSoftLimit(SoftLimitDirection.kReverse, Constants.Arm.SoftStop.ARM_RIGHT); // kReverse
  }

  public void resetDefault() {
    System.out.println("Default restored ********************************************");
    arm.configFactoryDefault(0);
    sideMotor.restoreFactoryDefaults();
    disableArmSoftLimit();;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // System.out.println(armMotorEncoder.getPosition());
    // System.out.println(armMotor.getAppliedOutput());
    // System.out.println(armMotorEncoder.getVelocity());
    // System.out.println(getArmCurrent());
    // System.out.println(armMotorEncoder.getVelocity());
    // System.out.println("Arm Current:" + getCurrent());
    SmartDashboard.putNumber("Side Motor Current ", getSideCurrent());
    // SmartDashboard.putNumber("Arm Motor Current: ", getArmCurrent());
    // System.out.println("Arm Encoder: " + getArmEncoder());
    // System.out.println("Side Encoder: " + getSideEncoder());
    // SmartDashboard.putNumber("velocity", sideMotorEncoder.getVelocity());
    SmartDashboard.putNumber("Arm Encoder: ", getEncoderCount());
    SmartDashboard.putNumber("Arm Current", getCurrent());
    SmartDashboard.putNumber("Arm Voltage", arm.getMotorOutputVoltage());
    SmartDashboard.putNumber("Pivot Encoder", getSideEncoder());
    // System.out.println(getArmCurrent() + " " + armMotor.getBusVoltage());
    
    // SmartDashboard.putNumber("side open ramp rate", sideMotor.getOpenLoopRampRate());
    // SmartDashboard.putNumber("side closed ramp rate", sideMotor.getClosedLoopRampRate());
    // SmartDashboard.putNumber("arm open ramp rate", armMotor.getOpenLoopRampRate());
    // SmartDashboard.putNumber("arm closed ramp rate", armMotor.getClosedLoopRampRate());
    SmartDashboard.putNumber("Side Motor Speed: ", sideMotor.get());
    SmartDashboard.putNumber("Arm Motor SPeed", arm.getMotorOutputPercent());
    // AbsoluteEncoder absEncoder;

    // absEncoder.getZeroOffset();

    // absEncoder.getPosition();
    
  }
}
