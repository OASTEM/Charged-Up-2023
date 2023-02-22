// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import edu.wpi.first.apriltag.AprilTag;
// import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.LogitechGamingPad;
//import frc.robot.utils.ShuffleBoard;

import java.io.IOException;
import java.nio.file.Path;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AprilTagDetect;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.Balance;
import frc.robot.commands.MoveArm;
import frc.robot.commands.MoveArmJoystick;
import frc.robot.commands.MoveArmUp;
import frc.robot.commands.Music;
import frc.robot.commands.PivotLeft;
import frc.robot.commands.PivotRight;
import frc.robot.commands.SetArmPosition;
import frc.robot.commands.SetPivotPosition;
import frc.robot.commands.Calibration.CalibrationFile;
import frc.robot.commands.Calibration.CalibrationSequence;
import frc.robot.commands.auto.StraightAuto;
import frc.robot.utils.ShuffleBoard;
//import frc.robot.commands.FollowPath;
import frc.robot.subsystems.Arm;

// import frc.robot.commands.Balance;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Manipulator;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //Controllers
  LogitechGamingPad pad = new LogitechGamingPad(0);
  LogitechGamingPad opPad = new LogitechGamingPad(1);
 

  //Subsytems
  private final DriveTrain driveTrain = new DriveTrain();
  private final Arm arm = new Arm();
  private final Manipulator manipulator = new Manipulator();
  private final ShuffleBoard shuffleboard = new ShuffleBoard(arm, manipulator);
  //private final Limelight limelight = new Limelight();
  //Commands


  //Buttons

  private final JoystickButton padA = new JoystickButton(pad, 1);
  private final JoystickButton padB = new JoystickButton(pad, 2);
  private final JoystickButton padX = new JoystickButton(pad, 3);
  private final JoystickButton padY = new JoystickButton(pad, 4);
  private final JoystickButton rightBumper = new JoystickButton(pad, 6);

  private final JoystickButton opPadA = new JoystickButton(opPad, 1);
  private final JoystickButton opPadB = new JoystickButton(opPad, 2);
  private final JoystickButton opPadX = new JoystickButton(opPad, 3);
  private final JoystickButton opPadY = new JoystickButton(opPad, 4);

  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    driveTrain.setDefaultCommand(new ArcadeDrive(driveTrain, pad));
    arm.setDefaultCommand(new MoveArmJoystick(arm, shuffleboard, opPad));
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    padX.whileTrue(new Balance(driveTrain));

    //padB.whileTrue(new Music(driveTrain));
    opPadB.whileTrue(new PivotLeft(arm, shuffleboard));
    rightBumper.onTrue(new InstantCommand(driveTrain::toggleSlowMode));
    opPadX.whileTrue(new PivotRight(arm, shuffleboard));
    opPadY.whileTrue(new MoveArm(arm, shuffleboard));
    opPadA.whileTrue(new MoveArmUp(arm,shuffleboard));
    //padA.whileTrue(new AprilTagDetect(limelight));
    // Configure your button bindings here
    padA.whileTrue(new SetArmPosition(arm, 73));
    // padA.onTrue(new SequentialCommandGroup( new SetPivotPosition(arm, -177), new SetArmPosition(arm, 73)));
    padB.whileTrue(new SetPivotPosition(arm,-100));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  //TODO AutoCommand to be returned
  public Command getAutonomousCommand(Trajectory trajectory, TrajectoryConfig config) {
    // final Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
    //   Arrays.asList(new Pose2d(), new Pose2d(1.0, 0, new Rotation2d())),
    //     config
    //   );
    //  final Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
    //    new Pose2d(0, 0, new Rotation2d(0)),
    //    List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
    //    new Pose2d(3, 0, new Rotation2d(0)),
    //    config
    //    );
    config = new TrajectoryConfig(Units.feetToMeters(2) , Units.feetToMeters(2));
    config.setKinematics(driveTrain.getKinematics());
    
    RamseteCommand command = new RamseteCommand(
    trajectory,
    driveTrain::getPose,
    new RamseteController(2.0, 0.7),
    driveTrain.getFeedForward(),
    driveTrain.getKinematics(),
    driveTrain::getSpeeds,
    driveTrain.getLeftPIDController(),
    driveTrain.getRightPIDController(),
    driveTrain::setOutput, 
    driveTrain
    );

    //return command;
    return new StraightAuto(driveTrain, arm);
  }

  public Command Music(){
    return new Music(driveTrain);
  }

  public Command Calibrate(){
    return new CalibrationSequence(driveTrain, arm, manipulator);
  }

}
