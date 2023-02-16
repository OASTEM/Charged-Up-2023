// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Manipulator;
import frc.robot.utils.ShuffleBoard;
import frc.robot.utils.Constants.openCloseMotor;

public class OpenClaw extends CommandBase {
  /** Creates a new OpenClaw. */
  private Manipulator manipulator;
  private ShuffleBoard shuffleboard;
  public OpenClaw(Manipulator manipulator, ShuffleBoard shuffleboard) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(manipulator);
    this.manipulator = manipulator;
    this.shuffleboard = shuffleboard;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    manipulator.setPID(shuffleboard.getOpenClosePID());
    manipulator.setManipulatorVelocity(100);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    manipulator.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
