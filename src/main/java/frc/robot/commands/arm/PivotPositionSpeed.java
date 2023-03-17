// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.utils.Constants;

public class PivotPositionSpeed extends CommandBase {
  /** Creates a new PivotPositionSpeed. */
  private Arm arm;
  private double position;
  private boolean done;
  private double error;
  public PivotPositionSpeed(Arm arm, double position) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
    this.arm = arm;
    this.position = position;
    done = false;
    error = 0;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(arm.getSideEncoder()<position){
      arm.setSide(0.3);
    }
    else{
      arm.setSide(-0.3);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.setSide(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    error = Math.abs(position - arm.getSideEncoder());
    if(error<=Constants.Arm.PIVOT_THRESH){
      return true;
    }
    return false;
  }
}
