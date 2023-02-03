package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utils.LogitechGamingPad;

public class ArcadeDrive extends CommandBase {
  DriveTrain driveTrain;
  LogitechGamingPad drivePad;

  double leftPad;
  double rightPad;

  public ArcadeDrive(DriveTrain driveTrain, LogitechGamingPad drivePad) {
    addRequirements(driveTrain);
    this.driveTrain = driveTrain;
    this.drivePad = drivePad;
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    leftPad = drivePad.getLeftAnalogYAxis();
    rightPad = drivePad.getRightAnalogXAxis();

    if (driveTrain.getSlowMode()){
      driveTrain.arcadeDrive((rightPad * -Constants.DriveTrain.SLOW_MODE),
          (leftPad * -Constants.DriveTrain.SLOW_MODE));
      SmartDashboard.putBoolean("Slow Mode: ", true);
    } else {
      driveTrain.arcadeDrive((rightPad * -Constants.DriveTrain.REGULAR_MODE_TURN),
          (leftPad * -Constants.DriveTrain.REGULAR_MODE));
      SmartDashboard.putBoolean("Slow Mode: ", false);
    }

  }

  @Override
  public void end(boolean interrupted) {
    driveTrain.stop();
  }


  @Override
  public boolean isFinished() {
    return false;
  }
}
