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

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private CANSparkMax armMotor;
  private SparkMaxPIDController armMotorPIDController;
  private RelativeEncoder armMotorEncoder;

  private CANSparkMax sideMotor;
  private SparkMaxPIDController sideMotorPIDController;
  private RelativeEncoder sideMotorEncoder;

  


  public Arm() {
    armMotor = new CANSparkMax(Constants.CANIDS.ARM_ID, MotorType.kBrushless);
    sideMotor = new CANSparkMax(Constants.CANIDS.SIDEARM_ID, MotorType.kBrushless);    

    armMotorPIDController = armMotor.getPIDController();
    armMotorEncoder = armMotor.getEncoder();



    armMotorPIDController.setP(Constants.Arm.upPID.p);
    armMotorPIDController.setI(Constants.Arm.upPID.i);
    armMotorPIDController.setD(Constants.Arm.upPID.d);

    sideMotorPIDController = sideMotor.getPIDController();
    sideMotorEncoder = sideMotor.getEncoder();

    sideMotorPIDController.setP(Constants.Arm.sidePID.p);
    sideMotorPIDController.setI(Constants.Arm.sidePID.i);
    sideMotorPIDController.setD(Constants.Arm.sidePID.d);
  }

  //Not Side Arm
  public void setArmMotorPosition(double position){
    armMotorPIDController.setReference(position, CANSparkMax.ControlType.kPosition);  
  }
  public void resetEncoders() {
    armMotorEncoder.setPosition(0);
  }

  public void setVelocity(double velocity) {
    armMotorPIDController.setReference(velocity, CANSparkMax.ControlType.kVelocity);
  }

  public void setPID(PID pid){
    armMotorPIDController.setP(pid.p);
    armMotorPIDController.setI(pid.i);
    armMotorPIDController.setD(pid.d);
  }

  public void setArm(double speed){
    armMotor.set(speed);
  }



  //Side Arm
  public void setSideMotorPosition(double position){
    sideMotorPIDController.setReference(position, CANSparkMax.ControlType.kPosition);  
  }
  public void resetSideEncoders() {
    sideMotorEncoder.setPosition(0);
  }
  public void setSide(double speed){
    sideMotor.set(speed);
  }
  public void setSideVelocity(int velocity) {
    sideMotorPIDController.setReference(velocity, CANSparkMax.ControlType.kVelocity);
  }


  public void setSidePID(PID pid){
    sideMotorPIDController.setP(pid.p);
    sideMotorPIDController.setI(pid.i);
    sideMotorPIDController.setD(pid.d);
  }
  public double getArmCurrent(){
    return Math.abs(armMotor.getOutputCurrent());
  }

  

  public double getSideCurrent(){
    return Math.abs(sideMotor.getOutputCurrent());
  }

  //Stop
  public void stop(){
    armMotor.stopMotor();
    sideMotor.stopMotor();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //System.out.println(armMotorEncoder.getPosition());
    //System.out.println(armMotor.getAppliedOutput());
    //System.out.println(armMotorEncoder.getVelocity());
    //System.out.println(getArmCurrent());
    // System.out.println(armMotorEncoder.getVelocity());
    SmartDashboard.putNumber("velocity", armMotorEncoder.getVelocity());
  }
}

