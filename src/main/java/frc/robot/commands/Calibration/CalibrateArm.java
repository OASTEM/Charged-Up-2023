// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Calibration;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Manipulator;

public class CalibrateArm extends CommandBase {
  /** Creates a new CalibrateArm. */
  private Arm arm;
  private boolean armDone;
  private Timer timer;
  private Timer timer2;
  private boolean changed;
  private Manipulator manipulator;

  public CalibrateArm(Arm arm, Manipulator manipulator) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm, manipulator);
    this.arm = arm;
    this.manipulator = manipulator;
    armDone = false;
    timer = new Timer();
    timer2 = new Timer();
    changed = false;
  }

  // Called when the command is initially scheduled.

  @Override
  public void initialize() {
    // System.out.println("Calibrate arm starting
    // *************************************");
    timer.reset();
    timer.start();
    timer2.reset();
    arm.ArmSoftLimit(false);
    manipulator.resetEncoders();
    arm.setArm(0.2);
    armDone = false;
    changed = false;
  }

  // Called every time the scheduler runs while the command is scheduled.

  @Override

  public void execute() {
    // System.out.println(arm.getArmCurrent());
    if(timer.get()>0.2 && !changed){
      // System.out.println("Got to calibrate Arm **************************");
      // System.out.println(arm.armLimit() +
      // "******************************************");
      arm.setArm(-0.2);
      timer2.start();
      changed = true;
    }
    if(timer2.get()>0.2 && arm.getArmCurrent()>=30){
      armDone = true;
      arm.setArm(0);
      System.out.println("GOT TO SECOND TIMER FOR CALIBRATE ARM **********************");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.stop();
    timer.stop();
    timer2.stop();
    timer.reset();
    timer2.reset();
    arm.resetEncoders();
    arm.disableArmSoftLimit();
    System.out.println("Caibrate arm is now done ***************************");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return armDone;
  }
}
