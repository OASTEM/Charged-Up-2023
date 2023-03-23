// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Pivot;
import frc.robot.utils.Constants;

public class PivotPositionSpeed extends CommandBase {
  /** Creates a new PivotPositionSpeed. */
  private double position;
  private double error;
  private Pivot pivot;
  public PivotPositionSpeed(Arm arm, double position, Pivot pivot) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pivot);
    this.pivot = pivot;
    this.position = position;
    error = 0;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(pivot.getSideEncoder()<position){
      pivot.setSide(0.3);
    }
    else{
      pivot.setSide(-0.3);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pivot.setSide(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    error = Math.abs(position - pivot.getSideEncoder());
    if(error<=Constants.Arm.PIVOT_THRESH){
      return true;
    }
    return false;
  }
}
