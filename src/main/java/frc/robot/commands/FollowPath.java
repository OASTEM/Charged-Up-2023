// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import com.pathplanner.lib.PathPlannerTrajectory;
// import frc.robot.subsystems.DriveTrain;


// public class FollowPath extends CommandBase {
//   private DriveTrain drivetrain;
//   private PathPlannerTrajectory trajectory;
//   private boolean initialPath;
//   /** Creates a new FollowPath. */
//   public FollowPath(PathPlannerTrajectory trajectory, DriveTrain drivetrain, boolean initialPath) {
//     super(
//         trajectory,
//         drivetrain::getPose,
//         drivetrain.getAutoXController(),
//         drivetrain.getAutoYController(),
//         drivetrain.getAutoThetaController(),
//         drivetrain);

//     this.drivetrain = drivetrain;
//     this.trajectory = trajectory;
//     this.initialPath = initialPath;
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {}

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
