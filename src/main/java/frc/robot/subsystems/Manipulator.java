// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;

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

    openCloseMotorPIDController.setP(Constants.Grabber.PID.P);
    openCloseMotorPIDController.setI(Constants.Grabber.PID.I);
    openCloseMotorPIDController.setD(Constants.Grabber.PID.D);
    
  }

  public void intake(double speed){
    leftMotor.set(speed);
    rightMotor.set(-speed);
  }

  public void open(){
    openCloseMotorEncoder.setPosition(0);
  }

  public void close(){
    openCloseMotorEncoder.setPosition(1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
