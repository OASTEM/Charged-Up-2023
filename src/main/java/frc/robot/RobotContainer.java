// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import edu.wpi.first.apriltag.AprilTag;
// import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.LogitechGamingPad;
import frc.robot.utils.NavX;
import frc.robot.utils.ShuffleBoard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AprilTagDetect;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.Balance;
import frc.robot.commands.Calibration;
import frc.robot.commands.MoveArm;
import frc.robot.commands.MoveArmUp;
import frc.robot.commands.Music;
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


  //Subsytems
  private final DriveTrain driveTrain = new DriveTrain();
  private final NavX navX = new NavX();
  private final Arm arm = new Arm();
  private final Manipulator manipulator = new Manipulator();
  private final ShuffleBoard shuffleboard = new ShuffleBoard();
  //private final Limelight limelight = new Limelight();
  //Commands


  //Buttons

  private final JoystickButton padA = new JoystickButton(pad, 1);
  private final JoystickButton padB = new JoystickButton(pad, 2);
  private final JoystickButton padX = new JoystickButton(pad, 3);
  private final JoystickButton padY = new JoystickButton(pad, 4);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
   //driveTrain.setDefaultCommand(new ArcadeDrive(driveTrain, pad));
   //arm.setDefaultCommand(new MoveArm(arm, pad));
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
    padA.whileTrue(new Balance(driveTrain, navX));
    //padB.whileTrue(new Music(driveTrain));
    padB.whileTrue(new MoveArmUp(arm, shuffleboard));
    //padX.onTrue(new InstantCommand(driveTrain::toggleSlowMode));
    padX.whileTrue(new MoveArm(arm, shuffleboard));
    padY.whileTrue(new Calibration(arm));
    //padA.whileTrue(new AprilTagDetect(limelight));
    // Configure your button bindings here
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  //TODO AutoCommand to be returned
  public Command getAutonomousCommand() {
    return null;
  }

  public Command Music(){
    return new Music(driveTrain);
  }
  public Command Calibrate(){
    return new Calibration(arm);
  }
}
