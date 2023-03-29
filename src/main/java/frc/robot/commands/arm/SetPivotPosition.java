// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pivot;
import frc.robot.utils.Constants;

public class SetPivotPosition extends CommandBase {
  private double position;
  private double error;
  private int count;
  private Pivot pivot;

  /** Creates a new setArmPosition. */
  public SetPivotPosition(Pivot pivot, double position) {
    addRequirements(pivot);
    this.position = position;
    this.pivot = pivot;
    error = 0;
    // timer =
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // if(arm.getArmEncoder()<40 && arm.getSideEncoder() < 2){
    pivot.setSidePID(Constants.Arm.sidePID);
    System.out.println("SETING PIVOT POSITION INITIALIZE ***************************");
    pivot.setSideMotorPosition(position);
    // }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    error = Math.abs(pivot.getSideEncoder() - position);
    if (error <= Constants.Arm.PIVOT_TOL) {
      count++;
    } else {
      count = 0;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pivot.stop();
    System.out.println("ENDING");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (count > 10);
  }
}
