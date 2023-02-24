// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manipulator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Manipulator;
import frc.robot.utils.LogitechGamingPad;
import frc.robot.utils.ShuffleBoard;

public class OpenCloseClaw extends CommandBase {
  /** Creates a new OpenCloseClaw. */
  private Manipulator manipulator;
  private ShuffleBoard shuffleBoard;
  private LogitechGamingPad gamepad;

  public OpenCloseClaw(Manipulator manipulator, ShuffleBoard shuffleboard, LogitechGamingPad gamepad) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(manipulator);
    this.manipulator = manipulator;
    this.shuffleBoard = shuffleboard;
    this.gamepad = gamepad;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double triggerValue = gamepad.getRightTriggerValue() - gamepad.getLeftTriggerValue();
    manipulator.setOC(triggerValue);
    if (triggerValue > 0) {
      manipulator.intake(.5); //should probably move to constant later on
    }
    else if (triggerValue < 0) {
      manipulator.intake(-.5); //outtake might not be neccesary for opening claw
    }
  }

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
