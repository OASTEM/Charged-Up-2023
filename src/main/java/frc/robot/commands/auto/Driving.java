// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utils.PID;


public class Driving extends CommandBase {
  DriveTrain driveTrain;
  
  private double leftkP;
  private double rightkP;
  private double setpoint = 0;
  private double leftError = 0;
  private double rightError = 0;
  private double leftSpeed = 0;
  private double rightSpeed = 0;
  private int count = 0;
  private double turningError;
  //private Encoder encoder = new Encoder(0, 1, false, EncodingType.k4X);


  /** Creates a new Driving. */
  public Driving(DriveTrain driveTrain, double setpoint, double leftkP, double rightkP) {
    addRequirements(driveTrain);
    this.driveTrain = driveTrain;
    this.setpoint = setpoint; 
    this.leftkP = leftkP;
    this.rightkP = rightkP;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.reset();
    driveTrain.resetEncoders();
    leftError = setpoint - driveTrain.getInchesFromNativeUnits(driveTrain.getLeftEncoderCount());
    rightError = setpoint - driveTrain.getInchesFromNativeUnits(driveTrain.getRightEncoderCount());
    leftSpeed = leftkP * leftError;
    rightSpeed = rightkP * rightError;
    turningError = driveTrain.getXAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turningError = driveTrain.getXAngle() * 0.01;
    SmartDashboard.putNumber("Turning Error", turningError);
    leftError = setpoint - driveTrain.getInchesFromNativeUnits(driveTrain.getLeftEncoderCount());
    rightError = setpoint - driveTrain.getInchesFromNativeUnits(driveTrain.getRightEncoderCount());
    leftSpeed = leftkP * leftError * (1 + turningError); //*1.05 */
    rightSpeed = rightkP * rightError * (1 - turningError);
    System.out.println(rightSpeed);
    driveTrain.tankDrive(leftSpeed, rightSpeed);
    if (Math.abs(leftError) < 
    100 && Math.abs(rightError) < 100){
      count++;
    }
    else{
      count = 0;
    }

    SmartDashboard.putNumber("Left Speed", leftSpeed);
    SmartDashboard.putNumber("Right Speed", rightSpeed);
    SmartDashboard.putNumber("Left Error", leftError);
    SmartDashboard.putNumber("Right Error", rightError);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stop();
    // driveTrain.printEncoders();
    //driveTrain.printInches();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return count >= 10;
  }
}