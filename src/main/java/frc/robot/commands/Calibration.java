// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class Calibration extends CommandBase {
  /** Creates a new Calibration. */
  private Arm arm;
  private Timer timer;
  private boolean armDone;
  public Calibration(Arm arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
    this.arm = arm;
    armDone = false;
    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    arm.set(-0.2);
    armDone = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(timer.get()>0.2){
      if(arm.getArmCurrent()>=25){
          armDone = true;
          arm.set(0);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    arm.stop();
    arm.resetEncoders();
    System.out.println("Calibration Stopped");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return armDone;
  }
}
