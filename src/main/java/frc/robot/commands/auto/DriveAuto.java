// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import frc.robot.utils.ShuffleBoard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Balance;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveAuto extends SequentialCommandGroup {
  /** Creates a new DriveAuto. */
  public DriveAuto(DriveTrain driveTrain, ShuffleBoard shuffleboard) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new tankStraight(driveTrain, 132, 0.025),
      new TankBack(driveTrain, -36, 0.025),
      new Balance(driveTrain, shuffleboard)
    );
  }
}
