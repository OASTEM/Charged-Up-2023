package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.Constants;
import frc.robot.subsystems.DriveTrain;

public class DriveStraight extends CommandBase {
  private final DriveTrain driveTrain;
  private double suppliedGoal;
  int count = 0;

  public DriveStraight(DriveTrain driveTrain, double suppliedGoal) {
    addRequirements(driveTrain);
    this.driveTrain = driveTrain;
    this.suppliedGoal = driveTrain.getNativeUnitsFromInches(suppliedGoal);
  }

  @Override
  public void initialize() {
    driveTrain.resetEncoders();
    driveTrain.setPosition(suppliedGoal);
    Timer.delay(.1);
  }

  @Override
  public void execute() {
    double leftError = Math.abs(suppliedGoal - driveTrain.getLeftEncoderCount());
    double rightError = Math.abs(suppliedGoal + driveTrain.getRightEncoderCount());
    if ((leftError <= Constants.DriveTrain.ERROR_THRESHOLD) && (rightError <= Constants.DriveTrain.ERROR_THRESHOLD)) {
      count++;
    } else {
      count = 0;
    }
  }

  @Override
  public void end(boolean interrupted) {
    driveTrain.stop();
    // driveTrain.printEncoders();
    // driveTrain.printInches();
  }

  @Override
  public boolean isFinished() {
    return count >= 10;

  }
}
