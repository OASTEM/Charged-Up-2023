// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.utils.Constants;
import frc.robot.utils.LogitechGamingPad;
import frc.robot.utils.PID;
import frc.robot.utils.ShuffleBoard;

public class MoveArmJoystick extends CommandBase {
  /** Creates a new MoveArm. */
  private Arm arm;
  private ShuffleBoard shuffleboard;
  private LogitechGamingPad drivePad;

  // public MoveArm(Arm arm, LogitechGamingPad drivePad){
  //   addRequirements(arm);
  //   this.arm = arm;
  //   this.drivePad = drivePad;
  // }
  public MoveArmJoystick(Arm arm, ShuffleBoard shuffleboard, LogitechGamingPad drivePad) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
    this.arm = arm;
    this.shuffleboard = shuffleboard;
    this.drivePad = drivePad;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if(arm.getArmEncoder() < Constants.Arm.SoftStop.ARM_UP){
    //   if(drivePad.getRightAnalogYAxis() < 0){
    //   arm.setArm(drivePad.getRightAnalogYAxis());
    //   // do nothing
    //   } //right moves counterclockwise
    // }
    // if(arm.getArmEncoder() >= Constants.Arm.SoftStop.ARM_DOWN){
    //   if(drivePad.getRightAnalogYAxis() > 0){
    //   arm.setArm(drivePad.getRightAnalogYAxis());

    //   } //right moves counterclockwise
    // }

    // if(arm.getSideEncoder() < Constants.Arm.SoftStop.ARM_LEFT){
    //   if(drivePad.getLeftAnalogXAxis() < 0){
    //   arm.setSide(-drivePad.getLeftAnalogXAxis());
    //   } //right moves counterclockwise
    // }
    // if(arm.getSideEncoder() >= Constants.Arm.SoftStop.ARM_RIGHT){
    //   if(drivePad.getLeftAnalogXAxis() > 0){
    //   arm.setSide(-drivePad.getLeftAnalogXAxis());
    //   } //right moves counterclockwise
    // }
    // if(arm.getSideEncoder() > 1000){
    //   arm.setSide(drivePad.getLeftAnalogXAxis()); 
    // }
    System.out.println(arm.getArmEncoder() + " ARM ***********");
    arm.setArm(drivePad.getRightAnalogYAxis() * 0.5);
    arm.setSide(-drivePad.getLeftAnalogXAxis() * 0.2);

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
