// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.utils.Constants;
// import edu.wpi.first.apriltag.AprilTag;
// import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.LogitechGamingPad;
//import frc.robot.utils.ShuffleBoard;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.NotifierCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.Balance;
import frc.robot.commands.Music;
import frc.robot.commands.Calibration.CalibrationSequence;
import frc.robot.commands.arm.MoveArmJoystick;
import frc.robot.commands.arm.SetArmPosition;
import frc.robot.commands.arm.SetPivotPosition;
import frc.robot.commands.auto.ArmBottomStartPosition;
import frc.robot.commands.auto.Driving;
import frc.robot.commands.auto.NoBalanceAuto;
import frc.robot.commands.auto.PreloadNoMove;
import frc.robot.commands.auto.StraightAuto;
import frc.robot.commands.manipulator.OpenClaw;
import frc.robot.commands.manipulator.GrabCone;
import frc.robot.commands.manipulator.GrabCube;
import frc.robot.utils.ShuffleBoard;
//import frc.robot.commands.FollowPath;
import frc.robot.subsystems.Arm;

// import frc.robot.commands.Balance;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Manipulator;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // Controllers
  LogitechGamingPad pad = new LogitechGamingPad(0);
  LogitechGamingPad opPad = new LogitechGamingPad(1);

  // Subsytems
  private final DriveTrain driveTrain = new DriveTrain();
  private final Arm arm = new Arm();
  private final Manipulator manipulator = new Manipulator();
  private final ShuffleBoard shuffleboard = new ShuffleBoard(arm, manipulator);
  // private final Limelight limelight = new Limelight();
  // Commands

  // Buttons

  private final JoystickButton padA = new JoystickButton(pad, 1);
  private final JoystickButton padB = new JoystickButton(pad, 2);
  private final JoystickButton padX = new JoystickButton(pad, 3);
  private final JoystickButton padY = new JoystickButton(pad, 4);
  private final JoystickButton rightBumper = new JoystickButton(pad, 6);
  private final JoystickButton leftBumper = new JoystickButton(pad, 5);

  private final JoystickButton opPadA = new JoystickButton(opPad, 1);
  private final JoystickButton opPadB = new JoystickButton(opPad, 2);
  private final JoystickButton opPadX = new JoystickButton(opPad, 3);
  private final JoystickButton opPadY = new JoystickButton(opPad, 4);
  private final JoystickButton opLeftBumper = new JoystickButton(opPad, 5);
  private final JoystickButton opStart = new JoystickButton(opPad, 8);
  private final JoystickButton opRightBumper = new JoystickButton(opPad, 6);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    driveTrain.setDefaultCommand(new ArcadeDrive(driveTrain, pad));
    arm.setDefaultCommand(new MoveArmJoystick(arm, shuffleboard, opPad));
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    padX.whileTrue(new Balance(driveTrain, shuffleboard));

    // padB.whileTrue(new Music(driveTrain));
    // opPadB.whileTrue(new PivotLeft(arm, shuffleboard));
    rightBumper.onTrue(new InstantCommand(driveTrain::toggleSlowMode));
    // opPadX.whileTrue(new PivotRight(arm, shuffleboard));
    // opPadY.whileTrue(new MoveArm(arm, shuffleboard));
    // opPadA.whileTrue(new MoveArmUp(arm,shuffleboard));
    opPadY.onTrue(new SetArmPosition(arm, Constants.Arm.ARM_SCORING_POSITION));
    opPadA.onTrue(new SetArmPosition(arm, Constants.Arm.ARM_START_POSITION));
    opPadX.onTrue(new SetPivotPosition(arm, -175));
    opPadB.onTrue(new SetPivotPosition(arm, -52));
    // opPadX.onTrue(new InstantCommand(manipulator::resetEncoders));
    opRightBumper.onTrue(new GrabCube(manipulator));
    opLeftBumper.onTrue(new GrabCone(manipulator));
    // padA.whileTrue(new AprilTagDetect(limelight));
    // Configure your button bindings here
    // padA.whileTrue(new SetArmPosition(arm, 73));
    opStart.whileTrue(new OpenClaw(manipulator));

    // padY.whileTrue(new ArmBottomStartPosition(arm));
    // padA.onTrue(new Driving(driveTrain, 170, 0.00338, 0.00352));
    // padA.onTrue(new DriveStraight(driveTrain, 50));
    // padB.whileTrue(new SetPivotPosition(arm,-100));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // TODO AutoCommand to be returned
  public Command getAutonomousCommand(Trajectory trajectory, TrajectoryConfig config) {
    // final Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
    // Arrays.asList(new Pose2d(), new Pose2d(1.0, 0, new Rotation2d())),
    // config
    // );
    // final Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
    // new Pose2d(0, 0, new Rotation2d(0)),
    // List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
    // new Pose2d(3, 0, new Rotation2d(0)),
    // config
    // );
    config = new TrajectoryConfig(Units.feetToMeters(2), Units.feetToMeters(2));
    config.setKinematics(driveTrain.getKinematics());

    // return command;
    // return new PreloadNoMove(driveTrain, arm, manipulator, shuffleboard);
    // return null;
    // return new StraightAuto(driveTrain, arm, manipulator, shuffleboard);
    return new NoBalanceAuto(driveTrain, arm, manipulator, shuffleboard);
  }

  public Command Music() {
    return new Music(driveTrain);
  }

  public Command Calibrate() {
    return new CalibrationSequence(driveTrain, arm, manipulator);
  }

}
