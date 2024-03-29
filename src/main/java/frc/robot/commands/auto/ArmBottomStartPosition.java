// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.SetArmPosition;
import frc.robot.commands.arm.SetPivotPosition;
import frc.robot.utils.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Pivot;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmBottomStartPosition extends SequentialCommandGroup {
  /** Creates a new ArmBottomStartPosition. */
  public ArmBottomStartPosition(Arm arm, Pivot pivot) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    System.out.println("EXECUTING THE STARTING POSITION IN CALIBRATE********************");
    addCommands(
      new SetPivotPosition(pivot, Constants.Arm.PIVOT_START_POSITION)
      .andThen(new SetArmPosition(arm, Constants.Arm.ARM_START_POSITION))
    );
  }
}
