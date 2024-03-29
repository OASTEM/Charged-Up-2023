// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.utils.Constants;

public class SetArmPosition extends CommandBase {
  private Arm arm;
  private double position;
  private double error;

  /** Creates a new setArmPosition. */
  public SetArmPosition(Arm arm, double position) {
    addRequirements(arm);
    this.arm = arm;
    this.position = position;
    error = 0;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(arm.getEncoderCount()<=position){
      arm.selectProfile(1);;
      System.out.println("Down PID****************************************************");
    }
    else{
      arm.selectProfile(0);
      System.out.println("Up PID****************************************************");
    }
    arm.setPosition(position);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    error = Math.abs(arm.getEncoderCount() - position);
    if (error <= Constants.Arm.ARM_TOL) {
    } else {
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // arm.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    // return (count > 10);
  }
}
