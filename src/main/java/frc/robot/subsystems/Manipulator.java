// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
import frc.robot.utils.PID;

public class Manipulator extends SubsystemBase {
  /** Creates a new Manipulator. */
  private CANSparkMax leftMotor;
  private CANSparkMax rightMotor;
  private CANSparkMax openCloseMotor;

  private SparkMaxPIDController leftMotorPIDController;
  private RelativeEncoder leftMotorEncoder;

  private SparkMaxPIDController rightMotorPIDController;
  private RelativeEncoder rightMotorEncoder;

  private SparkMaxPIDController openCloseMotorPIDController;
  private RelativeEncoder openCloseMotorEncoder;

  public Manipulator() {
    leftMotor = new CANSparkMax(Constants.CANIDS.LEFTGRABBER_ID, MotorType.kBrushless);
    rightMotor = new CANSparkMax(Constants.CANIDS.RIGHTGRABBER_ID, MotorType.kBrushless);
    openCloseMotor = new CANSparkMax(Constants.CANIDS.OPENCLOSE_ID, MotorType.kBrushless);

    leftMotor.setInverted(true);
    rightMotor.setInverted(false);

    // leftMotorPIDController = leftMotor.getPIDController();
    // leftMotorEncoder = leftMotor.getEncoder();

    // leftMotorPIDController.setP(Constants.Grabber.PID.P);
    // leftMotorPIDController.setI(Constants.Grabber.PID.I);
    // leftMotorPIDController.setD(Constants.Grabber.PID.D);

    // rightMotorPIDController = rightMotor.getPIDController();
    // rightMotorEncoder = rightMotor.getEncoder();

    // rightMotorPIDController.setP(Constants.Grabber.PID.P);
    // rightMotorPIDController.setI(Constants.Grabber.PID.I);
    // rightMotorPIDController.setD(Constants.Grabber.PID.D);

    openCloseMotorPIDController = openCloseMotor.getPIDController();
    openCloseMotorEncoder = openCloseMotor.getEncoder();

    openCloseMotorPIDController.setP(Constants.Grabber.PID.p);
    openCloseMotorPIDController.setI(Constants.Grabber.PID.i);
    openCloseMotorPIDController.setD(Constants.Grabber.PID.d);
    
  }

  public void stop(){
    openCloseMotor.stopMotor();
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }

  public void setOC(double speed){
    openCloseMotor.set(speed);
  }

  public void intake(double speed){
    leftMotor.set(speed);
    rightMotor.set(speed);
  }

  public void open(){
    openCloseMotorPIDController.setReference(0, CANSparkMax.ControlType.kPosition);
  }

  // public void close

  // open: -50
  // cone: -1.15
  //cube: 

//TODO: cube and cone

  public void setPosition(int position) {
    openCloseMotorEncoder.setPosition(position);
  }

  public void close(){
    openCloseMotorPIDController.setReference(Constants.openCloseMotor.closePosition, CANSparkMax.ControlType.kPosition);
  }
  public void setPID(PID pid){
    openCloseMotorPIDController.setP(pid.p);
    openCloseMotorPIDController.setI(pid.i);
    openCloseMotorPIDController.setD(pid.d);
  }

  public void setManipulatorVelocity(int velocity){
    leftMotorPIDController.setReference(velocity, CANSparkMax.ControlType.kVelocity);
    rightMotorPIDController.setReference(velocity, CANSparkMax.ControlType.kVelocity);
  }
  
  public void setOpenCloseMotor(int velocity){
    openCloseMotorPIDController.setReference(velocity, CANSparkMax.ControlType.kVelocity);
  }

  public void getCone(){
    openCloseMotorPIDController.setReference(Constants.openCloseMotor.conePosition, CANSparkMax.ControlType.kPosition);
    
  }

  public double getOCcurrent(){
    return openCloseMotor.getOutputCurrent();
  }

  public void resetEncoders(){
    openCloseMotorEncoder.setPosition(0);
  }

  // public void setManipulatorRightVelocity(int velocity){


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    getOpenCloseEncoder();
  }

  public void getOpenCloseEncoder(){
    SmartDashboard.putNumber("Open Close Encoder", openCloseMotor.getEncoder().getPosition());
  }
}