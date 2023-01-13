package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utils.LogitechGamingPad;

public class ArcadeDrive extends CommandBase {
  DriveTrain driveTrain;
  LogitechGamingPad drivePad;

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
    if (driveTrain.climbing()) {
      driveTrain.arcadeDrive((drivePad.getRightAnalogXAxis() * -Constants.DriveTrain.SUPER_SLOW_MODE),
          (drivePad.getLeftAnalogYAxis() * -Constants.DriveTrain.SUPER_SLOW_MODE));
      SmartDashboard.putBoolean("Slow Mode: ", true);
    } else if (driveTrain.getSlowMode()){
      driveTrain.arcadeDrive((drivePad.getRightAnalogXAxis() * -Constants.DriveTrain.SLOW_MODE),
          (drivePad.getLeftAnalogYAxis() * -Constants.DriveTrain.SLOW_MODE));
      SmartDashboard.putBoolean("Slow Mode: ", true);
    } else {
      driveTrain.arcadeDrive((drivePad.getRightAnalogXAxis() * -Constants.DriveTrain.REGULAR_MODE_TURN),
          (drivePad.getLeftAnalogYAxis() * -Constants.DriveTrain.REGULAR_MODE));
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
