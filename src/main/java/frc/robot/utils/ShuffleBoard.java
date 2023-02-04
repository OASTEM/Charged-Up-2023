// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Manipulator;

/** Add your docs here. */
public class ShuffleBoard {
    private Arm arm;
    private Manipulator manipulator;
    private PID armUpPID = Constants.Arm.upPID;
    private PID armDownPID = Constants.Arm.downPID;

    public void ShuffleBoard(Arm arm, Manipulator manipulator){
        this.arm = arm;
        this.manipulator = manipulator;
        SmartDashboard.putNumber("Arm Up P", armUpPID.p); 
        SmartDashboard.putNumber("Arm Up I", armUpPID.i);
        SmartDashboard.putNumber("Arm Up D", armUpPID.d);
        SmartDashboard.putNumber("Arm Up F", armUpPID.f);

        SmartDashboard.putNumber("Arm Down P", armDownPID.p); 
        SmartDashboard.putNumber("Arm Down I", armDownPID.i);
        SmartDashboard.putNumber("Arm Down D", armDownPID.d);
        SmartDashboard.putNumber("Arm Down F", armDownPID.f);
    }

    public PID getArmUpPID(){
        armUpPID.updatePID(
            SmartDashboard.getNumber("Arm Up P", armUpPID.p), 
            SmartDashboard.getNumber("Arm Up I", armUpPID.i),
            SmartDashboard.getNumber("Arm Up D", armUpPID.d),
            SmartDashboard.getNumber("Arm Up F", armUpPID.f));
        return armUpPID;
    }

    public PID getArmDownPID(){
        armDownPID.updatePID(
            SmartDashboard.getNumber("Arm Down P", armDownPID.p), 
            SmartDashboard.getNumber("Arm Down I", armDownPID.i),
            SmartDashboard.getNumber("Arm Down D", armDownPID.d),
            SmartDashboard.getNumber("Arm Down F", armDownPID.f));
        return armDownPID;
    }

}
