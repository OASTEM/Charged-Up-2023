// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.json.*;

import edu.wpi.first.networktables.NetworkTableEntry;

//import javax.swing.text.StyleContext.SmallAttributeSet;

import edu.wpi.first.networktables.NetworkTableInstance;
//import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Limelight extends SubsystemBase {
  private double id = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(0);
  private double x = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
  private double y = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
  private double a = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
  private double v = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
  // String fID;
  // private double t6r_fs =
  // NetworkTableInstance.getDefault().getTable("limelight").getEntry("json").getDouble(0.0);;
  private double limelightHeight;
  private double goalHeight;
  private double limelightMountDegree;

  private double angleToGoalDegrees;
  private double angleToGoalInRadians;
  private double distance;

  String jsonString;
  int fID;

  /** Creates a new Limelight. */
  public Limelight() {

  }

  @Override
  public void periodic() {
    // System.out.println(NetworkTableInstance.getDefault().getTable("limelight").getEntry("json").getString("None"));
    jsonString = NetworkTableInstance.getDefault().getTable("limelight").getEntry("json").getString("None");
    // System.out.println(jsonString);
    JSONObject obj = new JSONObject(jsonString);
    JSONArray fiducial = obj.getJSONObject("Results").getJSONArray("Fiducial");
    try {
      JSONObject arrayID = fiducial.getJSONObject(0);
      fID = arrayID.getInt("fID");
    } catch (Exception e) {
      fID = -1;
      System.out.println("error with fID");
    }

    System.out.println(fID);
    // JSONArray arr = obj.getJSONArray("fID"); // notice that `"posts": [...]`
    // for (int i = 0; i < arr.length(); i++)
    // {
    // fID = arr.getJSONObject(i).getString("fID");
    // }
    // System.out.println(fID);

    this.id = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(0);
    this.x = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    this.y = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    this.a = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
    this.v = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);

    // this.t6r_fs =
    // NetworkTableInstance.getDefault().getTable("limelight").getEntry("t6r_fs").getDouble(0.0);;
    SmartDashboard.putNumber("tx", x);
    SmartDashboard.putNumber("ty", y);
    SmartDashboard.putNumber("ta", a);
    SmartDashboard.putNumber("tv", v);
    SmartDashboard.putNumber("id", id);
    // SmartDashboard.putNumber("field", t6r_fs);
    // This method will be called once per scheduler run
  }

  public String getJson() {
    return jsonString;
  }

  public double getDistance() {
    angleToGoalDegrees = limelightMountDegree + y;
    angleToGoalInRadians = Math.toRadians(angleToGoalDegrees);
    distance = (goalHeight - limelightHeight) / Math.tan(angleToGoalInRadians);
    return distance;
  }

  public double getLimelightAngle() {
    double length = 119;
    // In inches adjacent
    double angle = (goalHeight - limelightHeight) / length;
    // Gets the ratio of opp and adj
    angle = Math.atan(angle);
    // Takes this ratio and returns the angle
    angle = Math.abs(angle - Math.toRadians(y));
    // Accounts for any off center
    angle = Math.toDegrees(angle);
    // Converts it to degrees
    return angle;
  }

  public double getXAngle() {
    return x;
  }
}
