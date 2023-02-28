// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;


public class Driving extends CommandBase {
  DriveTrain driveTrain;

  double leftkP;
  double rightkP;
  double setpoint = 0;
  double leftError = 0;
  double rightError = 0;
  double leftSpeed = 0;
  double rightSpeed = 0;
  int count = 0;

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
    driveTrain.resetEncoders();
    leftError = setpoint - driveTrain.getInchesFromNativeUnits(driveTrain.getLeftEncoderCount());
    rightError = setpoint - driveTrain.getInchesFromNativeUnits(driveTrain.getRightEncoderCount());
    leftSpeed = leftkP * leftError;
    rightSpeed = rightkP * rightError;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    leftError = setpoint - driveTrain.getInchesFromNativeUnits(driveTrain.getLeftEncoderCount());
    rightError = setpoint - driveTrain.getInchesFromNativeUnits(driveTrain.getRightEncoderCount());
    leftSpeed = leftkP * leftError; //*1.05 */
    rightSpeed = rightkP * rightError;
    System.out.println(rightSpeed);
    driveTrain.tankDrive(leftSpeed, rightSpeed);
    if (Math.abs(leftError) < 1 && Math.abs(rightError) < 1){
      count++;
    }
    else{
      count = 0;
    }
    SmartDashboard.putNumber("Right Speed", rightSpeed);
    SmartDashboard.putNumber("Left Speed", leftSpeed);
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