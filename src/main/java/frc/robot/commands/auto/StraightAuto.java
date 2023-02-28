// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Balance;
import frc.robot.commands.arm.SetArmPosition;
import frc.robot.commands.arm.SetPivotPosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utils.Constants;
import frc.robot.utils.ShuffleBoard;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StraightAuto extends SequentialCommandGroup {
  /** Creates a new StraightAuto. */
  public StraightAuto(DriveTrain driveTrain, Arm arm, ShuffleBoard shuffleboard) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetArmPosition(arm, Constants.Arm.ARM_SCORING_POSITION).withTimeout(2)
      .andThen(new SetPivotPosition(arm, Constants.Arm.PIVOT_START).withTimeout(3)), 
      // new SetArmPosition(arm, 33).withTimeout(2)
      //Close Claw (Claw is closed) -Averi :D   //0.00448
    
      new Driving(driveTrain, 260, 0.00278, 0.00352).withTimeout(4.5), //0.00588, 0.00652
      new Driving(driveTrain, -150, 0.00688, 0.00352).withTimeout(3),
      new Balance(driveTrain, shuffleboard)
    );
  }

  //TODO:
  /*arm 75 pivot -175
   * Start arm height: 27
   * Start pivot: -72
   * start pivot other side: -52
   * middle pivot: -177
   * down arm: 73
   * encoder for driving straight
   */
}
