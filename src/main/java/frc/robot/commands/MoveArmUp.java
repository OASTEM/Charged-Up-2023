// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.utils.LogitechGamingPad;
import frc.robot.utils.PID;
import frc.robot.utils.ShuffleBoard;

public class MoveArmUp extends CommandBase {
  /** Creates a new MoveArm. */
  private Arm arm;
  private double p, i, d;
  private double velocity;
  private ShuffleBoard shuffleboard;
  //private LogitechGamingPad drivePad;

  // public MoveArm(Arm arm, LogitechGamingPad drivePad){
  //   addRequirements(arm);
  //   this.arm = arm;
  //   this.drivePad = drivePad;
  // }
  public MoveArmUp(Arm arm, ShuffleBoard shuffleboard) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
    this.arm = arm;
    this.shuffleboard = shuffleboard;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.resetEncoders();
    //arm.setArmMotorPosition(100);
    // p = SmartDashboard.getNumber("PArmUp", 0.0001); 
    // i = SmartDashboard.getNumber("IArmUp", 0.000001);
    // d = SmartDashboard.getNumber("DArmUp", 0);
    // SmartDashboard.putNumber("PArmUp", p);
    // SmartDashboard.putNumber("IArmUp", i);
    // SmartDashboard.putNumber("DArmUp", d);
    // PID pid = new PID(p, i, d,0);
    // arm.setPID(pid);
    velocity = SmartDashboard.getNumber("velocityArm", -1920);
    SmartDashboard.putNumber("velocityArm", velocity);
    arm.setPID(shuffleboard.getArmUpPID());
    //arm.setVelocity(-1920);
    arm.setVelocity(velocity);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //arm.moveArm(drivePad.getRightAnalogXAxis());
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
