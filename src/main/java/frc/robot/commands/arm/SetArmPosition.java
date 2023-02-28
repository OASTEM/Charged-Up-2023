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
  /** Creates a new setArmPosition. */
  public SetArmPosition(Arm arm, double position) {
    addRequirements(arm);
    this.arm = arm;
    this.position = position;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.setPID(Constants.Arm.upPID);
    arm.setArmMotorPosition(position);
    
    System.out.println("HOWAIDHDAAHIUAGBIJUHIUADHGIUADHIUHOAIUDHOUADHOU");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if(arm.getArmEncoder()>40 && arm.getSideEncoder() < 2){
    //   arm.setSideSoftLimit();
    // }
    // else{
    //   arm.setArmSoftLimit();
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("ENDING SeT ARP POSITION ***********************************");
    arm.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}