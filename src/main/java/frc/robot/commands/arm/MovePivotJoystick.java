// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Pivot;
import frc.robot.utils.LogitechGamingPad;
import frc.robot.utils.ShuffleBoard;

public class MovePivotJoystick extends CommandBase {
  /** Creates a new MoveArm. */
  private LogitechGamingPad drivePad;
  private Pivot pivot;
  private Arm arm;

  public MovePivotJoystick(ShuffleBoard shuffleboard, LogitechGamingPad drivePad, Pivot pivot, Arm arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pivot);
    this.drivePad = drivePad;
    this.pivot = pivot;
    this.arm = arm;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if(Math.abs(drivePad.getLeftAnalogYAxis()) <= 0.1){
    // arm.setArmMotorPosition(arm.getArmEncoder());
    // }
    // else
    // {
    if (!arm.allowPivot() && drivePad.getDPad()==-1){
      pivot.setSide(0);
      return;
    }
    
    if(drivePad.getDPad() > -1){
      if(drivePad.checkDPad(2)){
        pivot.setSide(0.14);
      }
      else if(drivePad.checkDPad(6)){
        pivot.setSide(-0.14);
      }
    }

    else{
    pivot.setSide(-drivePad.getRightAnalogXAxis() * 0.35);
  }
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

// 32 arm for stradle front
// 24 arm for stradle back