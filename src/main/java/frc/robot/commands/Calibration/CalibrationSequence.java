// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Calibration;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.ArmBottomStartPosition;
import frc.robot.commands.manipulator.GrabCone;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Manipulator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CalibrationSequence extends ParallelCommandGroup {
  /** Creates a new CalibrationSequence. */
  DriveTrain driveTrain;
  Arm arm;
  Manipulator manipulator;
  public CalibrationSequence(DriveTrain driveTrain, Arm arm, Manipulator manipulator) {
    this.driveTrain = driveTrain; 
    this.arm = arm;
    this.manipulator = manipulator;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //TODO Ramp auto so that it doesnt go as fast at the start
      new CalibrateDriveTrain(driveTrain),
      new SequentialCommandGroup(
        new InstantCommand(arm::resetDefault),
        new InstantCommand(arm::setArmRampRate),
        new InstantCommand(arm::setPivotRampRate),
        // new InstantCommand(arm::disableArmSoftLimit),
        new CalibrateArm(arm, manipulator),
        new CalibratePivot(arm),
        // new InstantCommand(arm::resetEncoders),
        // new InstantCommand(arm::resetSideEncoders),
        // new InstantCommand(arm::enableArmSoftLimit),
        // new InstantCommand(arm::setArmSoftLimit),
        // new SetPivotPosition(arm, -180).withTimeout(4)
        // .andThen(new SetArmPosition(arm, 68))
        new InstantCommand(arm::disableArmSoftLimit).withTimeout(0.1),
        new ArmBottomStartPosition(arm),
        new InstantCommand(arm::enableArmSoftLimit).withTimeout(0.1),
        new InstantCommand(arm::setArmSoftLimit).withTimeout(0.1),
        new GrabCone(manipulator).withTimeout(2)
        // new InstantCommand(manipulator::getCone),
        // new InstantCommand(manipulator::stopOpenClose)
        //Commands.runOnce(() -> arm.ArmSoftLimit(true))
        // new SetPivotPosition(arm, -180),
        // new SetArmPosition(arm, 68)
      )
    );
  }
}
