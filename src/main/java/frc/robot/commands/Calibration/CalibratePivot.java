// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Calibration;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Pivot;

public class CalibratePivot extends CommandBase {
  /** Creates a new CalibratePivot. */
  private Arm arm;
  private boolean pivotDone;
  private Timer timer;
  private Timer timer2;
  private Pivot pivot;

  // private Manipulator manipulator;
  public CalibratePivot( Pivot pivot) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pivot);

    this.pivot = pivot;
    pivotDone = false;
    timer = new Timer();
    timer2 = new Timer();
    // this.manipulator = manipulator;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    timer2.reset();
    pivot.setSide(-0.1);
    pivotDone = false;
    System.out.println("***************** Calibrate is starting");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.get() > .5) {
      pivot.setSide(0.2);
      timer2.start();
    }
    if (timer2.get() > 0.2 && pivot.getSideCurrent() >= 10) { // 7
      pivotDone = true;
      pivot.setSide(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pivot.resetSideEncoders();
    timer.stop();
    timer.reset();
    timer2.reset();
    pivot.setSide(0);

    //arm.enableArmSoftLimit();
    //TODO Reenable soft limits with correct values once ready
    // manipulator.resetEncoders();
    System.out.println("Calibrate pivot done now ***************************");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pivotDone;
  }
}
