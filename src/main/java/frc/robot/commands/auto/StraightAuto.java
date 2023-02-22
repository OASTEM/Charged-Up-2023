// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Driving;
import frc.robot.commands.SetArmPosition;
import frc.robot.commands.SetPivotPosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utils.Constants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StraightAuto extends SequentialCommandGroup {
  /** Creates a new StraightAuto. */
  private DriveTrain driveTrain;
  private Arm arm;
  public StraightAuto(DriveTrain driveTrain, Arm arm) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetArmPosition(arm, Constants.Arm.ARM_START_POSITION),
      new SetPivotPosition(arm, Constants.Arm.PIVOT_START),
      new Driving(driveTrain, 100, 0)
    );
  }

  //TODO:
  /*arm 75 pivot -175
   * Start arm height: 33
   * Start pivot: -72
   * start pivot other side: -52
   * middle pivot: -177
   * down arm: 73
   * encoder for driving straight
   */
}
