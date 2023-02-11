// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Manipulator;

public class Calibration extends CommandBase {
  /** Creates a new Calibration. */
  private Arm arm;
  private Manipulator manipulator;
  private Timer timer;
  private DriveTrain driveTrain;
  private boolean armDone;
  private boolean armPivot;
  private boolean manipulatorDone;
  private boolean frontLTest;
  private boolean backLTest;
  private boolean frontRTest;
  private boolean backRTest;

  public Calibration(Arm arm, Manipulator manipulator, DriveTrain driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm, manipulator, driveTrain);
    this.arm = arm;
    this.driveTrain = driveTrain;
    this.manipulator = manipulator;
    armDone = false;
    armPivot = false;
    manipulatorDone = false;
    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.addInstruments();
    driveTrain.loadMusic();
    timer.reset();
    timer.start();
    arm.setArm(0.2);
    arm.setSide(0.2);
    manipulator.setOC(0.2);
    manipulatorDone = false;
    armPivot = false;
    armDone = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if((driveTrain.playBackL() && driveTrain.playFrontL() && driveTrain.playBackR() && driveTrain.playFrontR()) == false){
      System.out.println("Drivetrain issue");
    }
    if(timer.get()>0.2){
      manipulator.setOC(-0.2);
      arm.setArm(-0.1);
      arm.setSide(0.1);
      if(arm.getArmCurrent()>=25){
          armDone = true;
          arm.setArm(0);
      }
      if(arm.getSideCurrent()>=25){
        armPivot = true;
        arm.setSide(0);
      }
      if(manipulator.getOCcurrent() >= 25 && !manipulatorDone){
        manipulatorDone = true;
        manipulator.setOC(0);
        manipulator.resetEncoders();
      }
      if(manipulatorDone){
        manipulator.getCone();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    arm.stop();
    arm.resetEncoders();
    driveTrain.stopMusic();
    System.out.println("Calibration Stopped");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return armDone && armPivot;
  }
}
