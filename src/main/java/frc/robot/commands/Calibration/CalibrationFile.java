// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Calibration;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Manipulator;

public class CalibrationFile extends CommandBase {
  /** Creates a new Calibration. */
  private Arm arm;
  private Manipulator manipulator;
  private Timer timer;
  private Timer pivotTimer;
  private DriveTrain driveTrain;
  private boolean armDone;
  private boolean armPivot;
  private boolean driveTrainDone;
  private boolean manipulatorDone;
  private boolean frontLTest;
  private boolean backLTest;
  private boolean frontRTest;
  private boolean backRTest;

  public CalibrationFile(Arm arm, Manipulator manipulator, DriveTrain driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm, manipulator, driveTrain);
    this.arm = arm;
    this.driveTrain = driveTrain;
    this.manipulator = manipulator;
    armDone = false;
    armPivot = false;
    manipulatorDone = false;
    timer = new Timer();
    pivotTimer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.addInstruments();
    driveTrain.loadMusic();
    timer.reset();
    timer.start();
    pivotTimer.reset();
    arm.setArm(0.2);
    //arm.setSide(0.2);
    // manipulator.setOC(0.2);
    // manipulatorDone = false;
    armPivot = false;
    driveTrainDone = false;
    armDone = false;
    System.out.println("In Calibration");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if((driveTrain.playBackL() && driveTrain.playFrontL() && driveTrain.playBackR() && driveTrain.playFrontR()) == false){
      System.out.println("Drivetrain issue");
      driveTrainDone = true;
      armDone = true;
      armPivot = true;
      //Play another sound to signify drivetrain error
    }
    else{
      if(timer.get()>5){
        System.out.println("DriveTrain done");
        driveTrainDone = true;
      }
    }

    if(timer.get()>0.3 && !armDone){
      //manipulator.setOC(-0.2);
      arm.setArm(-0.2);
      if(arm.getArmCurrent()>=25){
          armDone = true;
          arm.setArm(0);
      }
      
      // if(manipulator.getOCcurrent() >= 25 && !manipulatorDone){
      //   manipulatorDone = true;
      //   manipulator.setOC(0);
      //   manipulator.resetEncoders();
      // }
      // if(manipulatorDone){
      //   manipulator.getCone();
      // }
    }
    else if(armDone){
      arm.setSide(-0.1);
      pivotTimer.start();
    }
    if(pivotTimer.get()>0.1 && !armPivot){
      System.out.println("Pivot first if statement");
      arm.setSide(0.1);
    }
    if(pivotTimer.get()>0.2 && arm.getSideCurrent()>=5 && !armPivot){
      armPivot = true;
      arm.setSide(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    pivotTimer.stop();
    arm.stop();
    arm.resetEncoders();
    driveTrain.stopMusic();
    System.out.println("Calibration Stopped");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return armDone && armPivot && driveTrainDone;
  }
}
