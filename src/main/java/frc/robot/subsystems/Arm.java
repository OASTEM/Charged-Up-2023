// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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

    System.out.println("CALLED ARM CONSTRUCTOR");

    armMotor.enableSoftLimit(SoftLimitDirection.kForward, false);
    armMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
    sideMotor.enableSoftLimit(SoftLimitDirection.kForward, false);
    sideMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);

    armMotorPIDController.setP(Constants.Arm.upPID.p);
    armMotorPIDController.setI(Constants.Arm.upPID.i);
    armMotorPIDController.setD(Constants.Arm.upPID.d);

    sideMotorPIDController = sideMotor.getPIDController();
    sideMotorEncoder = sideMotor.getEncoder();

    sideMotorPIDController.setP(Constants.Arm.sidePID.p);
    sideMotorPIDController.setI(Constants.Arm.sidePID.i);
    sideMotorPIDController.setD(Constants.Arm.sidePID.d);

    sideMotor.setClosedLoopRampRate(Constants.Arm.PIVOT_CLOSED_LOOP_RATE);
    sideMotor.setOpenLoopRampRate(Constants.Arm.PIVOT_OPEN_LOOP_RATE);
    armMotor.setClosedLoopRampRate(Constants.Arm.ARM_CLOSED_LOOP_RATE);
    armMotor.setOpenLoopRampRate(Constants.Arm.ARM_OPEN_LOOP_RATE);

    // sideMotor.setSmartCurrentLimit(30);
    // armMotor.setSmartCurrentLimit(40);
    // sideMotor.setSecondaryCurrentLimit(35);
    // armMotor.setSecondaryCurrentLimit(45);
    System.out.println("In Arm Constructor ************");
  }

  // Not Side Arm
  public void setArmMotorPosition(double position) {
    System.out.println("SETTIGN ARM MOTOR POISIOTN *****************************************************");
    armMotorPIDController.setReference(position, CANSparkMax.ControlType.kPosition);
  }

  public void resetEncoders() {
    armMotorEncoder.setPosition(0);
  }

  public void setVelocity(double velocity) {
    armMotorPIDController.setReference(velocity, CANSparkMax.ControlType.kVelocity);
  }

  public void setPID(PID pid) {
    armMotorPIDController.setP(pid.p);
    armMotorPIDController.setI(pid.i);
    armMotorPIDController.setD(pid.d);
    armMotorPIDController.setFF(pid.f);
  }

  public void setArm(double speed) {
    // System.out.println("Setting arm motor speed ***************************");
    armMotor.set(speed);
  }

  public double getArmEncoder() {
    return armMotor.getEncoder().getPosition();
  }

  // Side Arm
  public void setSideMotorPosition(double position) {
    // System.out.println("SETTING SIDE MOTOR POSITION
    // *********************************");
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

  public void setPivotPercent(double speed) {
    if (!(getArmEncoder() > Constants.Arm.ARM_LIMIT_BOTTOM && getSideEncoder() < Constants.Arm.ARM_LIMIT_LEFT
        && getSideEncoder() > Constants.Arm.ARM_LIMIT_RIGHT)) {
      sideMotor.set(speed);
    }
  }

  public void setSidePID(PID pid) {
    sideMotorPIDController.setP(pid.p);
    sideMotorPIDController.setI(pid.i);
    sideMotorPIDController.setD(pid.d);
  }

  public double getArmCurrent() {
    return Math.abs(armMotor.getOutputCurrent());
  }

  public double getSideEncoder() {
    return sideMotor.getEncoder().getPosition();
  }

  public double getSideCurrent() {
    return Math.abs(sideMotor.getOutputCurrent());
  }

  // Stop
  public void stop() {
    armMotor.stopMotor();
    sideMotor.stopMotor();
  }

  // public void ArmSoftLimit(boolean enable) {
  // armMotor.enableSoftLimit(SoftLimitDirection.kForward, enable);
  // armMotor.enableSoftLimit(SoftLimitDirection.kReverse, enable);
  // sideMotor.enableSoftLimit(SoftLimitDirection.kForward, enable);
  // sideMotor.enableSoftLimit(SoftLimitDirection.kReverse, enable);
  // }

  // public void ArmSoftLimit() {
  // armMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
  // armMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
  // sideMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
  // sideMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
  // }

  // public void setArmSoftLimit() {
  // armMotor.setSoftLimit(SoftLimitDirection.kForward,
  // Constants.Arm.SoftStop.ARM_DOWN);
  // armMotor.setSoftLimit(SoftLimitDirection.kReverse,
  // Constants.Arm.SoftStop.ARM_UP);
  // sideMotor.setSoftLimit(SoftLimitDirection.kForward,
  // Constants.Arm.SoftStop.ARM_LEFT);
  // sideMotor.setSoftLimit(SoftLimitDirection.kReverse,
  // Constants.Arm.SoftStop.ARM_RIGHT);
  // }

  // Forward
  // Reverse

  //Forward
  //Reverse

  public void setPivotRampRate(){
    sideMotor.setClosedLoopRampRate(Constants.Arm.PIVOT_CLOSED_LOOP_RATE);
    sideMotor.setOpenLoopRampRate(Constants.Arm.PIVOT_OPEN_LOOP_RATE);
  }

  public void setSideRange() {
    sideMotorPIDController.setOutputRange(-0.2, 0.2);
  }
  public void setArmRampRate(){
    armMotor.setClosedLoopRampRate(Constants.Arm.ARM_CLOSED_LOOP_RATE);
    armMotor.setOpenLoopRampRate(Constants.Arm.ARM_OPEN_LOOP_RATE);

  }

  public void ArmSoftLimit(boolean enable) {
    armMotor.enableSoftLimit(SoftLimitDirection.kForward, enable);
    armMotor.enableSoftLimit(SoftLimitDirection.kReverse, enable);
    sideMotor.enableSoftLimit(SoftLimitDirection.kForward, enable);
    sideMotor.enableSoftLimit(SoftLimitDirection.kReverse, enable);
  }

  public void enableArmSoftLimit() {
    armMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    armMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    sideMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    sideMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    System.out.println("Enableing soft limit ******************************************");
  }

  public void disableArmSoftLimit() {
    armMotor.enableSoftLimit(SoftLimitDirection.kForward, false);
    armMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
    sideMotor.enableSoftLimit(SoftLimitDirection.kForward, false);
    sideMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
  }

  public void setArmSoftLimit() {
    armMotor.setSoftLimit(SoftLimitDirection.kForward, Constants.Arm.SoftStop.ARM_DOWN); // kForward
    armMotor.setSoftLimit(SoftLimitDirection.kReverse, Constants.Arm.SoftStop.ARM_UP); // KReverse
    sideMotor.setSoftLimit(SoftLimitDirection.kForward, Constants.Arm.SoftStop.ARM_LEFT); // kForward
    sideMotor.setSoftLimit(SoftLimitDirection.kReverse, Constants.Arm.SoftStop.ARM_RIGHT); // kReverse
  }

  public void setSideSoftLimit() {
    sideMotor.setSoftLimit(SoftLimitDirection.kForward, Constants.Arm.SoftStop.PIVOT_MID_LEFT);
    sideMotor.setSoftLimit(SoftLimitDirection.kReverse, Constants.Arm.SoftStop.PIVOT_MID_RIGHT);
  }

  public void resetDefault() {
    System.out.println("Default restored ********************************************");
    armMotor.restoreFactoryDefaults();
    sideMotor.restoreFactoryDefaults();
    ArmSoftLimit(false);
  }

  public boolean armLimit() {
    return armMotor.isSoftLimitEnabled(SoftLimitDirection.kForward);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // System.out.println(armMotorEncoder.getPosition());
    // System.out.println(armMotor.getAppliedOutput());
    // System.out.println(armMotorEncoder.getVelocity());
    // System.out.println(getArmCurrent());
    // System.out.println(armMotorEncoder.getVelocity());
    SmartDashboard.putNumber("Side Motor Current ", getSideCurrent());
    SmartDashboard.putNumber("Arm Motor Current: ", getArmCurrent());
    // System.out.println("Arm Encoder: " + getArmEncoder());
    // System.out.println("Side Encoder: " + getSideEncoder());
    // SmartDashboard.putNumber("velocity", sideMotorEncoder.getVelocity());
    SmartDashboard.putNumber("Arm Encoder: ", getArmEncoder());
    SmartDashboard.putNumber("Pivot Encoder", getSideEncoder());
    System.out.println(getArmCurrent() + " " + armMotor.getBusVoltage());
    
    // SmartDashboard.putNumber("side open ramp rate", sideMotor.getOpenLoopRampRate());
    // SmartDashboard.putNumber("side closed ramp rate", sideMotor.getClosedLoopRampRate());
    // SmartDashboard.putNumber("arm open ramp rate", armMotor.getOpenLoopRampRate());
    // SmartDashboard.putNumber("arm closed ramp rate", armMotor.getClosedLoopRampRate());
  }
}
