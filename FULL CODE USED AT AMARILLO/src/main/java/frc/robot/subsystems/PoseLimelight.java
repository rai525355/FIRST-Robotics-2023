// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class PoseLimelight extends SubsystemBase {
  /** Creates a new Limelight. */
  private double redX;
  private double redY;
  private double redZ;
  private double redRoll;
  private double redPitch;
  private double redYaw;

  private double blueX;
  private double blueY;
  private double blueZ;
  private double blueRoll;
  private double bluePitch;
  private double blueYaw;

  private static long id;

  private double tx;

  public PoseLimelight() {
    redX = 1;
    redY = 0;
    redZ = 0;
    redRoll = 0;
    redPitch = 0;
    redYaw = 0;

    blueX = 1;
    blueY = 0;
    blueZ = 0;
    blueRoll = 0;
    bluePitch = 0;
    blueYaw = 0;
  }

  public double getRedX() {
    return redX;
  }

  public double getRedY() {
    return redY;
  }

  public double getRedZ() {
    return redZ;
  }

  public double getRedRoll() {
    return redRoll;
  }

  public double getRedPitch() {
    return redPitch;
  }

  public double getRedYaw() {
    return redYaw;
  }

  public double getBlueX() {
    return blueX;
  }

  public double getBlueY() {
    return blueY;
  }

  public double getBlueZ() {
    return blueZ;
  }

  public double getBlueRoll() {
    return blueRoll;
  }

  public double getBluePitch() {
    return bluePitch;
  }

  public double getBlueYaw() {
    return blueYaw;
  }

  public static long getId() {
    return id;
  }

  public double getTx() {
    return tx;
  }

  public void changeMode(int index) {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(index);
  }



  @Override
  public void periodic() {
  // This method will be called once per scheduler run

    // NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    // NetworkTableEntry tx = table.getEntry("tx");
    // NetworkTableEntry ty = table.getEntry("ty");
    // NetworkTableEntry ta = table.getEntry("ta");
      
    //read values periodically
    // double x = tx.getDouble(0.0);
    // double y = ty.getDouble(0.0);
    // double area = ta.getDouble(0.0);
      
    // post to smart dashboard periodically
    // SmartDashboard.putNumber("LimelightX", x);
    // SmartDashboard.putNumber("LimelightY", y);
    // SmartDashboard.putNumber("LimelightArea", area);




    double[] bbb = {1,2,3,4,5,6};
      
      
    // double[] arr = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_targetspace").getDoubleArray(bbb);
      
    // x = arr[0];
    // y = arr[1];
    // z = arr[2];
    // roll = arr[3];
    // pitch = arr[4];
    // yaw = arr[5];

    double[] redArr = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_wpired").getDoubleArray(bbb);
      
    redX = redArr[0];
    redY = redArr[1];
    redZ = redArr[2];
    redRoll = redArr[3];
    redPitch = redArr[4];
    redYaw = redArr[5];
      
    SmartDashboard.putNumber("Limelight RedX", redX);
    SmartDashboard.putNumber("Limelight RedY", redY);
    SmartDashboard.putNumber("Limelight RedZ", redZ);


    double[] blueArr = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_wpiblue").getDoubleArray(bbb);
      
    blueX = blueArr[0];
    blueY = blueArr[1];
    blueZ = blueArr[2];
    blueRoll = blueArr[3];
    bluePitch = blueArr[4];
    blueYaw = blueArr[5];
      
    SmartDashboard.putNumber("Limelight BlueX", blueX);
    SmartDashboard.putNumber("Limelight BlueY", blueY);
    SmartDashboard.putNumber("Limelight BlueZ", blueZ);

    tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);





    NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(0);

    id = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getInteger(0);
    SmartDashboard.putNumber("Tag ID", id);
    //System.out.println(id[0]);
    // System.out.println(id);

    // SmartDashboard.putNumber("Roll?", arr[3]);
    // SmartDashboard.putNumber("Pitch?", arr[4]);
    // SmartDashboard.putNumber("Yaw?", arr[5]);
  }
}