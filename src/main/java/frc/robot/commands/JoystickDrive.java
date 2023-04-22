// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utils.Constants;

public class JoystickDrive extends CommandBase {
  /** Creates a new JoystickDrive. */
  private DriveTrain driveTrain;
  private Joystick joystick;
  public JoystickDrive(DriveTrain driveTrain, Joystick joystick) {
    addRequirements(driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
    this.joystick = joystick;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (Math.abs())
    driveTrain.arcadeDrive(Math.pow(joystick.getX(), 3)*Constants.DriveTrain.SLOW_MODE, -Math.pow(joystick.getY(), 3)*Constants.DriveTrain.SLOW_MODE);
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
