// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Manipulator;

/** Add your docs here. */
public class ShuffleBoard {
    private PID armUpPID = Constants.Arm.upPID;
    private PID armDownPID = Constants.Arm.downPID;
    private PID armSidePID = Constants.Arm.sidePID;
    private PID openClosePID = Constants.openCloseMotor.openClosePID;
    private PID driveTrainPID = Constants.DriveTrain.PID;

    public ShuffleBoard(Arm arm, Manipulator manipulator) {
        SmartDashboard.putNumber("Arm Up P", armUpPID.p);
        SmartDashboard.putNumber("Arm Up I", armUpPID.i);
        SmartDashboard.putNumber("Arm Up D", armUpPID.d);
        SmartDashboard.putNumber("Arm Up F", armUpPID.f);

        SmartDashboard.putNumber("Arm Down P", armDownPID.p);
        SmartDashboard.putNumber("Arm Down I", armDownPID.i);
        SmartDashboard.putNumber("Arm Down D", armDownPID.d);
        SmartDashboard.putNumber("Arm Down F", armDownPID.f);

        SmartDashboard.putNumber("Arm Side P", armSidePID.p);
        SmartDashboard.putNumber("Arm Side I", armSidePID.i);
        SmartDashboard.putNumber("Arm Side D", armSidePID.d);
        SmartDashboard.putNumber("Arm Side F", armSidePID.f);

        SmartDashboard.putNumber("PBal", driveTrainPID.p);
        SmartDashboard.putNumber("IBal", driveTrainPID.i);
        SmartDashboard.putNumber("DBal", driveTrainPID.d);
        SmartDashboard.putNumber("FBal", driveTrainPID.f);

    }

    public PID getArmUpPID() {
        armUpPID.updatePID(
                SmartDashboard.getNumber("Arm Up P", armUpPID.p),
                SmartDashboard.getNumber("Arm Up I", armUpPID.i),
                SmartDashboard.getNumber("Arm Up D", armUpPID.d),
                SmartDashboard.getNumber("Arm Up F", armUpPID.f));
        return armUpPID;
    }

    public PID getArmDownPID() {
        armDownPID.updatePID(
                SmartDashboard.getNumber("Arm Down P", armDownPID.p),
                SmartDashboard.getNumber("Arm Down I", armDownPID.i),
                SmartDashboard.getNumber("Arm Down D", armDownPID.d),
                SmartDashboard.getNumber("Arm Down F", armDownPID.f));
        return armDownPID;
    }

    public PID getArmSidePID() {
        armSidePID.updatePID(
                SmartDashboard.getNumber("Arm Side P", armSidePID.p),
                SmartDashboard.getNumber("Arm Side I", armSidePID.i),
                SmartDashboard.getNumber("Arm Side D", armSidePID.d),
                SmartDashboard.getNumber("Arm Side F", armSidePID.f));
        return armSidePID;
    }

    public PID getOpenClosePID() {
        openClosePID.updatePID(
                SmartDashboard.getNumber("openClose P", openClosePID.p),
                SmartDashboard.getNumber("openClose I", openClosePID.i),
                SmartDashboard.getNumber("openClose D", openClosePID.d),
                SmartDashboard.getNumber("openClose F", openClosePID.f));
        return openClosePID;
    }

    // public PID getBalancePID(){
    // driveTrainPID.updatePID(
    // SmartDashboard.getNumber("PBal", driveTrainPID.p),
    // SmartDashboard.getNumber("IBal", driveTrainPID.i),
    // SmartDashboard.getNumber("DBal", driveTrainPID.d),
    // SmartDashboard.getNumber("FBal", driveTrainPID.f));
    // return driveTrainPID;
    // }

    // public PID getClosePID(){
    // closePID.updatePID(
    // SmartDashboard.getNumber("close P", closePID.p),
    // SmartDashboard.getNumber("close I", closePID.i),
    // SmartDashboard.getNumber("close D", closePID.d),
    // SmartDashboard.getNumber("close F", closePID.f));
    // return closePID;
    // }

    // public PID balPID(){

    // }

}
