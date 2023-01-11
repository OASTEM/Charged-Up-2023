// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utils.NavX;
import frc.robot.utils.PID;

public class Balance extends CommandBase {
  /** Creates a new balance. */
  DriveTrain driveTrain;
  NavX navX;
  private double error;
  private final double goal = -1;
  private final double maxEffort = 0.3;
  //PID Values need to be tuned
  //Might need to create two pid values for both sides of the drivetrain
  PID balancePID = new PID(0.025, 0, 0);

  public Balance(DriveTrain driveTrain, NavX navX) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
    this.driveTrain = driveTrain;
    this.navX = navX;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.stop();
    navX.reset();
    System.out.println("autdoaiuad");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Error is equal to the NavX getYaw (using getRoll)
    // driveTrain.setLeftSpeed(0.3);
    // driveTrain.setBackLeftSpeed();
    // driveTrain.setRightSpeed(0.3);
    this.error = navX.getXAngle();

    
    balancePID.calculate(this.goal, this.error);
    double effort = balancePID.getOutput();
    if (effort < -maxEffort) {
      effort = -maxEffort;
    } else if (effort > maxEffort) {
      effort = maxEffort;
    }

    driveTrain.setLeftSpeed(effort);
    driveTrain.setRightSpeed(effort);

    SmartDashboard.putNumber("navXYError", this.error);
    SmartDashboard.putNumber("PID Speed", effort);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
