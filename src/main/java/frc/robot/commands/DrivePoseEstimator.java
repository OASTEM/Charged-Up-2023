// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utils.NavX;

public class DrivePoseEstimator extends CommandBase {
  /** Creates a new DrivePoseEstimator. */
  DriveTrain driveTrain;
  DifferentialDriveKinematics drive;
  DifferentialDrivePoseEstimator estimator;
  Rotation2d rotation;
  Pose2d startingPose;
  NavX navX;
  public DrivePoseEstimator(DriveTrain driveTrain, NavX navX) {
    this.driveTrain = driveTrain;
    this.navX = navX;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);

    startingPose = new Pose2d();
    rotation = new Rotation2d(Math.toRadians(navX.getZAngle()));
    drive = new DifferentialDriveKinematics(0.5);
    estimator = new DifferentialDrivePoseEstimator(drive, rotation, driveTrain.getLeftEncoderCount(), driveTrain.getRightEncoderCount(), startingPose);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rotation = new Rotation2d(Math.toRadians(navX.getZAngle()));
    estimator.update(rotation, driveTrain.getLeftEncoderCount(), driveTrain.getRightEncoderCount());

    SmartDashboard.putNumber("X Position", estimator.getEstimatedPosition().getTranslation().getX());
    SmartDashboard.putNumber("Y Position", estimator.getEstimatedPosition().getTranslation().getY());
    SmartDashboard.putNumber("Rotation", estimator.getEstimatedPosition().getRotation().getDegrees());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    estimator.resetPosition(rotation, driveTrain.getLeftEncoderCount(), driveTrain.getRightEncoderCount(), startingPose);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
