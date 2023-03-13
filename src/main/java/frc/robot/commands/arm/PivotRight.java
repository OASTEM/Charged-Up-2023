// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.utils.ShuffleBoard;

public class PivotRight extends CommandBase {
  /** Creates a new Pivot. */
  private Arm arm;
  private ShuffleBoard shuffleboard;

  public PivotRight(Arm arm, ShuffleBoard shuffleboard) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
    this.arm = arm;
    this.shuffleboard = shuffleboard;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.setSidePID(shuffleboard.getArmSidePID());
    arm.setSide(0.4);
    // arm.setSideVelocity(10000);
    System.out.println("got to pivot right");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
