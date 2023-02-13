// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Calibration;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class CalibrateDriveTrain extends CommandBase {
  /** Creates a new CalibrateDriveTrain. */
  private DriveTrain driveTrain;
  private boolean driveTrainDone;
  public CalibrateDriveTrain(DriveTrain driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
    this.driveTrain = driveTrain;
    driveTrainDone=false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.addInstruments();
    driveTrain.loadMusic();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if((driveTrain.playBackL() && driveTrain.playFrontL() && driveTrain.playBackR() && driveTrain.playFrontR()) == false){
      System.out.println("Drivetrain issue");
      driveTrainDone = true;
      //Play another sound to signify drivetrain error
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stopMusic();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return driveTrainDone;
  }
}
