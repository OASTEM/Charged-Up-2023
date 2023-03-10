// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;
import frc.robot.utils.PID;

public class FollowAprilTag extends CommandBase {
  /** Creates a new FollowAprilTag. */
  DriveTrain driveTrain;
  Limelight limelight;
  PID followPID;
  double p, i, d;
  double distanceError;
  double angleError;

  public FollowAprilTag(DriveTrain driveTrain, Limelight limelight) {
    addRequirements(driveTrain, limelight);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    distanceError = limelight.getDistance();
    angleError = limelight.getXAngle();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
