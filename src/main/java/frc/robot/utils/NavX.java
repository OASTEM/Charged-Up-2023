// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.utils;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.I2C.Port;

/** Add your docs here. */
public class NavX {
    private final AHRS navX;
    public NavX(){
        navX = new AHRS(Port.kMXP, (byte) 50);
    }

    public double getZAngle() {
        return navX.getAngle();
      }

    public double getXAngle(){
        return navX.getRoll();
    }

    public double getYAngle(){
        return navX.getPitch();
    }
    
    public void reset() {
        navX.reset();
      }
}