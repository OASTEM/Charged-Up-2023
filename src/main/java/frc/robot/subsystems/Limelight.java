// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.lang.reflect.Array;
import org.json.*;

//import javax.swing.text.StyleContext.SmallAttributeSet;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
//import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Limelight extends SubsystemBase {
  private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  private NetworkTableEntry t6c_ts;
  private double id = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(0);
  private double x = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
  private double y = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
  private double a = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
  private double v = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
  String fID;
  //private double t6r_fs = NetworkTableInstance.getDefault().getTable("limelight").getEntry("json").getDouble(0.0);;

  String jsonString;
  /** Creates a new Limelight. */
  public Limelight() {

  }

  @Override
  public void periodic() {
    //System.out.println(NetworkTableInstance.getDefault().getTable("limelight").getEntry("json").getString("None"));
    jsonString = NetworkTableInstance.getDefault().getTable("limelight").getEntry("json").getString("None");
    // System.out.println(jsonString);
    JSONObject obj = new JSONObject(jsonString);
    JSONArray pageName = obj.getJSONObject("Results").getJSONArray("Fiducial");
    JSONObject arrayID = pageName.getJSONObject(0);
    System.out.println(arrayID);
    // JSONArray arr = obj.getJSONArray("fID"); // notice that `"posts": [...]`
    // for (int i = 0; i < arr.length(); i++)
    // {
    //     fID = arr.getJSONObject(i).getString("fID");
    // }
    // System.out.println(fID);
    

    this.id = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(0);
    this.x = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    this.y = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    this.a = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
    this.v = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    //this.t6r_fs = NetworkTableInstance.getDefault().getTable("limelight").getEntry("t6r_fs").getDouble(0.0);;
    SmartDashboard.putNumber("tx", x);
    SmartDashboard.putNumber("ty", y);
    SmartDashboard.putNumber("ta", a);
    SmartDashboard.putNumber("tv", v);
    SmartDashboard.putNumber("id", id);
    //SmartDashboard.putNumber("field", t6r_fs);
    // This method will be called once per scheduler run
  }

  public String getJson(){
    return jsonString;
  }
}
