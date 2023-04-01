// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
import frc.robot.utils.PID;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private TalonFX arm;
  //TODO: Encoder values (max and min for soft stop)
//TODO: PID values for side and arm
//TODO: Current values for calibration
//TODO: Clean up Code
  public Arm() {
    arm = new TalonFX(Constants.CANIDS.ARM_ID);
    System.out.println("CALLED ARM CONSTRUCTOR");
    resetDefault();

    arm.setNeutralMode(NeutralMode.Brake);

    arm.configPeakOutputForward(1);
    arm.configPeakOutputReverse(-1);

    // arm.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 8, 8, 2));

    initPIDController(Constants.Arm.upPID);

    configRampRates();

    // sideMotor.setSmartCurrentLimit(30);
    // armMotor.setSmartCurrentLimit(40);
    // sideMotor.setSecondaryCurrentLimit(35);
    // armMotor.setSecondaryCurrentLimit(45);
    System.out.println("In Arm Constructor ************");
    initPIDController(Constants.Arm.upPID);
    initPIDController(Constants.Arm.downPID);
  }
  
  public void initPIDController( PID pid){
    arm.config_kP(pid.s, pid.p);
    arm.config_kI(pid.s, pid.i);
    arm.config_kD(pid.s, pid.d);
    //arm.selectProfileSlot(0, 0);
  }

  public void selectProfile(int id){
    arm.selectProfileSlot(id, 0);
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

  // start of changed code
  public boolean allowPivot() {
      if (getEncoderCount() > 130000){ // whatever the value is for below drivetrain frame
        return false;
      }
    return true;
   }
  
  // end of changed code


  // Stop
  public void stop() {
    arm.set(ControlMode.PercentOutput, 0.0);
  }

  public void enableArmSoftLimit() {
    // arm.configForwardSoftLimitEnable(true, 0);
    // arm.configReverseSoftLimitEnable(true, 0);
    System.out.println("Enableing soft limit ******************************************");
  }

  public void disableArmSoftLimit() {
    // arm.configForwardSoftLimitEnable(false, 0);
    // arm.configReverseSoftLimitEnable(false, 0);
  }

  public void setArmSoftLimit() {
    arm.configForwardSoftLimitThreshold(Constants.Arm.SoftStop.ARM_UP, 0);
    arm.configReverseSoftLimitThreshold(Constants.Arm.SoftStop.ARM_DOWN, 0);
  }

  public void resetDefault() {
    System.out.println("Default restored ********************************************");
    arm.configFactoryDefault(0);
    disableArmSoftLimit();;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Encoder: ", getEncoderCount());
    SmartDashboard.putNumber("Arm Current", getCurrent());
    SmartDashboard.putNumber("Arm Voltage", arm.getMotorOutputVoltage());
    SmartDashboard.putNumber("Arm Motor SPeed", arm.getMotorOutputPercent());
    // AbsoluteEncoder absEncoder;
    // absEncoder.getZeroOffset();

    // absEncoder.getPosition();
    
  }
}
