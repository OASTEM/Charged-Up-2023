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

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private CANSparkMax armMotor;
  private SparkMaxPIDController armMotorPIDController;
  private RelativeEncoder armMotorEncoder;
  
  public Arm() {
    armMotor = new CANSparkMax(Constants.CANIDS.ARM_ID, MotorType.kBrushless);

    armMotorPIDController = armMotor.getPIDController();
    armMotorEncoder = armMotor.getEncoder();

    armMotorPIDController.setP(Constants.Arm.PID.P);
    armMotorPIDController.setI(Constants.Arm.PID.I);
    armMotorPIDController.setD(Constants.Arm.PID.D);

  }

  public void setArmMotorPosition(double position){
    armMotorPIDController.setReference(position, CANSparkMax.ControlType.kPosition);  
  }
  public void resetEncoders() {
    armMotorEncoder.setPosition(0);
  }

  public void moveArm(double speed){
    armMotor.set(speed);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
