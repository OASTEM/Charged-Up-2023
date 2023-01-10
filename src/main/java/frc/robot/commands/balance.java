// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utils.NavX;
import frc.robot.utils.PID;

public class Balance extends CommandBase {
  /** Creates a new balance. */
  DriveTrain driveTrain;
  NavX navX;
  private double error;
  private final double goal = 0;
  //PID Values need to be tuned
  //Might need to create two pid values for both sides of the drivetrain
  PID balancePID = new PID(0.05, 0, 0);

  public Balance(DriveTrain driveTrain, NavX navX) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
    this.driveTrain = driveTrain;
    this.navX = navX;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    navX.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Error is equal to the NavX getYaw
    this.error = navX.getYAngle();
    balancePID.calculate(this.goal, this.error);
    driveTrain.setLeftSpeed(balancePID.getOutput());
    driveTrain.setRightSpeed(balancePID.getOutput());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
