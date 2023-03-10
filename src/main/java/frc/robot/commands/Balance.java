// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utils.Constants;
import frc.robot.utils.PID;
import frc.robot.utils.ShuffleBoard;

public class Balance extends CommandBase {
  /** Creates a new balance. */
  DriveTrain driveTrain;
  private double error;
  private final double goal = 0;
  private final double maxEffort = 1;

  private double p, i, d;
  // PID Values need to be tuned
  // Might need to create two pid values for both sides of the drivetrain
  // PID balancePID = new PID(0.023, 0.002, 0.002);
  PID balancePID;

  public Balance(DriveTrain driveTrain, ShuffleBoard shuffleboard) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);

    this.driveTrain = driveTrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("STARTIGN TO BALANCE ***************************************");

    // p = SmartDashboard.getNumber("PBal",0.01015 );//0.01015
    // i = SmartDashboard.getNumber("IBal", 0);
    // d = SmartDashboard.getNumber("DBal", 0.0001);
    // SmartDashboard.putNumber("PBal", p);
    // SmartDashboard.putNumber("IBal", i);
    // SmartDashboard.putNumber("DBal", d);

    // balancePID = new PID(p, i, d, 0);
    balancePID = Constants.DriveTrain.PID;
    // balancePID = shuffleboard.getBalancePID();

    driveTrain.stop();
    driveTrain.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Error is equal to the NavX getYaw (using getRoll)
    // driveTrain.setLeftSpeed(0.3);
    // driveTrain.setBackLeftSpeed();
    // driveTrain.setRightSpeed(0.3);
    this.error = driveTrain.getXAngle();
    // System.out.println(error);
    double effort = balancePID.calculate(this.goal, this.error);
    if (effort < -maxEffort) {
      effort = -maxEffort;
    } else if (effort > maxEffort) {
      effort = maxEffort;
    }

    driveTrain.setLeftSpeed(-effort); // effort
    driveTrain.setRightSpeed(-effort); // effort

    SmartDashboard.putNumber("navXYError", this.error);
    SmartDashboard.putNumber("PID Speed", effort);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stop();
    p = SmartDashboard.getNumber("PBal", 0.021);
    i = SmartDashboard.getNumber("IBal", 0.002);
    d = SmartDashboard.getNumber("DBal", 0.002);
    balancePID = new PID(p, i, d, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
