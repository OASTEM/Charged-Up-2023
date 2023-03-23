// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class tankStraight extends CommandBase {
  /** Creates a new tankStraight. */
  private DriveTrain driveTrain;
  private double kP;
  double heading;
  double error;
  double distance;
  private double goal;
  boolean done = false;
  public tankStraight(DriveTrain driveTrain, double goal, double kP) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
    this.driveTrain = driveTrain;
    this.kP = kP;
    this.goal = driveTrain.getNativeUnitsFromInches(goal);
    done = false;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.resetEncoders();
    driveTrain.reset();
    heading = driveTrain.getZAngle();
    done = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    error = heading - driveTrain.getZAngle();
    distance = Math.abs(goal - driveTrain.getRightEncoderCount());
    driveTrain.tankDrive(.6 - kP * error, .6 + kP * error);
    // System.out.println(driveTrain.getInchesFromNativeUnits(driveTrain.getRightEncoderCount()));
    if(driveTrain.getRightEncoderCount()>=goal && driveTrain.getLeftEncoderCount()>=goal){
      done = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
