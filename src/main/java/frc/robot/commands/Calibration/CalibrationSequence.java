// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Calibration;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.SetArmPosition;
import frc.robot.commands.arm.SetPivotPosition;
import frc.robot.commands.manipulator.GrabCube;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Pivot;
import frc.robot.utils.Constants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CalibrationSequence extends ParallelCommandGroup {
  /** Creates a new CalibrationSequence. */
  DriveTrain driveTrain;
  Arm arm;
  Manipulator manipulator;
  Pivot pivot;

  public CalibrationSequence(DriveTrain driveTrain, Arm arm, Manipulator manipulator, Pivot pivot) {
    this.driveTrain = driveTrain;
    this.arm = arm;
    this.manipulator = manipulator;
    this.pivot = pivot;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //TODO Ramp auto so that it doesnt go as fast at the start
      new CalibrateDriveTrain(driveTrain),
      new SequentialCommandGroup(
        new InstantCommand(arm::configRampRates),
        new InstantCommand(pivot::setPivotRampRate),
        new CalibrateArm(arm, manipulator),
        new CalibratePivot( pivot),
        new InstantCommand(arm::resetEncoders),
        new InstantCommand(pivot::resetSideEncoders),
        new SetPivotPosition(pivot, Constants.Arm.PIVOT_START_POSITION)
        .andThen(new SetArmPosition(arm, Constants.Arm.ARM_START_POSITION)).withTimeout(4),
        // new InstantCommand(arm::enableArmSoftLimit),
        // new InstantCommand(arm::setArmSoftLimit)

      // new SetArmPosition(arm, 174798)


        // new InstantCommand(arm::disableArmSoftLimit).withTimeout(0.1),
        // new ArmBottomStartPosition(arm),
        // new InstantCommand(arm::enableArmSoftLimit).withTimeout(0.1),
        new InstantCommand(pivot::setArmSoftLimit),
        new InstantCommand(pivot::enableArmSoftLimit),
        new InstantCommand(arm::setArmSoftLimit).withTimeout(0.1),
        new GrabCube(manipulator).withTimeout(2)
        











        //Merge conflict
        // TODO Ramp auto so that it doesnt go as fast at the start
        // new CalibrateDriveTrain(driveTrain),
        // new SequentialCommandGroup(
        //     new InstantCommand(arm::resetDefault),
        //     new InstantCommand(arm::setArmRampRate),
        //     new InstantCommand(arm::setPivotRampRate),
        //     // new InstantCommand(arm::disableArmSoftLimit),
        //     new CalibrateArm(arm, manipulator),
        //     new CalibratePivot(arm),
        //     // new InstantCommand(arm::resetEncoders),
        //     // new InstantCommand(arm::resetSideEncoders),
        //     // new InstantCommand(arm::enableArmSoftLimit),
        //     // new InstantCommand(arm::setArmSoftLimit),
        //     // new SetPivotPosition(arm, -180).withTimeout(4)
        //     // .andThen(new SetArmPosition(arm, 68))
        //     new InstantCommand(arm::disableArmSoftLimit).withTimeout(0.1),
        //     new ArmBottomStartPosition(arm),
        //     new InstantCommand(arm::enableArmSoftLimit).withTimeout(0.1),
        //     new InstantCommand(arm::setArmSoftLimit).withTimeout(0.1),
        //     new GrabCone(manipulator).withTimeout(2)
        // new InstantCommand(manipulator::getCone),
        // new InstantCommand(manipulator::stopOpenClose)
        // Commands.runOnce(() -> arm.ArmSoftLimit(true))
        // new SetPivotPosition(arm, -180),
        // new SetArmPosition(arm, 68)
        ));
  }
}
