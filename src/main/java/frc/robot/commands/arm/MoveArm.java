// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.arm;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.Arm;
// import frc.robot.utils.ShuffleBoard;

// public class MoveArm extends CommandBase {
//   /** Creates a new MoveArm. */
//   private Arm arm;
//   private ShuffleBoard shuffleboard;
//   // private LogitechGamingPad drivePad;

//   // public MoveArm(Arm arm, LogitechGamingPad drivePad){
//   // addRequirements(arm);
//   // this.arm = arm;
//   // this.drivePad = drivePad;
//   // }
//   public MoveArm(Arm arm, ShuffleBoard shuffleboard) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     addRequirements(arm);
//     this.arm = arm;
//     this.shuffleboard = shuffleboard;

//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     arm.resetEncoders();
//     // arm.setArmMotorPosition(100);
//     // p = SmartDashboard.getNumber("PArm", 0.0002);
//     // i = SmartDashboard.getNumber("IArm", 0.000001);
//     // d = SmartDashboard.getNumber("DArm", 0.000021);
//     // SmartDashboard.putNumber("PArm", p);
//     // SmartDashboard.putNumber("IArm", i);
//     // SmartDashboard.putNumber("DArm", d);
//     // PID pid = new PID(p, i, d,0);
//     arm.setPID(shuffleboard.getArmDownPID());
//     arm.setVelocity(1920); // 1920
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     // arm.moveArm(drivePad.getRightAnalogXAxis());
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     arm.stop();
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
