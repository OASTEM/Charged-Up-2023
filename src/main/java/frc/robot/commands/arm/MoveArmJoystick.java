// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Pivot;
import frc.robot.utils.LogitechGamingPad;
import frc.robot.utils.ShuffleBoard;

public class MoveArmJoystick extends CommandBase {
  /** Creates a new MoveArm. */
  private Arm arm;
  private LogitechGamingPad drivePad;
  private Pivot pivot;
  // public MoveArm(Arm arm, LogitechGamingPad drivePad){
  // addRequirements(arm);
  // this.arm = arm;
  // this.drivePad = drivePad;
  // }
  public MoveArmJoystick(Arm arm, ShuffleBoard shuffleboard, LogitechGamingPad drivePad, Pivot pivot) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm, pivot);
    this.arm = arm;
    this.drivePad = drivePad;
    this.pivot = pivot;
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
    if(Math.abs(drivePad.getLeftAnalogYAxis()) <= .05){
      arm.setPosition(arm.getEncoderCount());
    }
    else{
      arm.setSpeed(drivePad.getLeftAnalogYAxis() * 0.2);
    }
    if (!arm.allowPivot()){
      pivot.setSide(0);
    }
    else{
      pivot.setSide(-drivePad.getRightAnalogXAxis() * 0.2);
    }
    
    System.out.println(drivePad.getRightAnalogXAxis());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // arm.setArmMotorPosition(arm.getArmEncoder());
    // arm.setSideMotorPosition(arm.getSideEncoder());
    arm.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

// 32 arm for stradle front
// 24 arm for stradle back