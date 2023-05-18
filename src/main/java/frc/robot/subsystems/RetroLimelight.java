// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class RetroLimelight extends SubsystemBase {
  /** Creates a new Limelight. */
  private double x;

  public RetroLimelight() {
    x = 1;
    changeMode(1);
  }

  public double getX() {
    return x;
  }

  // 0 = retroreflectve   1 = apriltags
  public void setPipeline(int index) {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(index);
  }

  // 0 = vision    1 = driver
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




    //double[] bbb = {1,2,3,4,5,6};
      
      
    //double[] arr = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_targetspace").getDoubleArray(bbb);
    x = NetworkTableInstance.getDefault().getTable("limelight2").getEntry("tx").getDouble(0);

   
      
    SmartDashboard.putNumber("RetroLimelight X", x);
    // SmartDashboard.putNumber("Limelight Y", y);
    // SmartDashboard.putNumber("Limelight Z", z);


    // limelight1 = NetworkTableInstance.getDefault().getTable("limelight1");
    // limelight2 = NetworkTableInstance.getDefault().getTable("limelight2");

    // NetworkTableEntry botpose = limelight1.getEntry("botpose_targetspace");
    // double[] arr = botpose.getDoubleArray(new double[6]);

    // x = arr[0];
    // y = arr[1];
    // z = arr[2];
    // roll = arr[3];
    // pitch = arr[4];
    // yaw = arr[5];

    //NetworkTableEntry txRetro = limelight2.getEntry("tx");

    // NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(0);

    // SmartDashboard.putNumber("Roll?", arr[3]);
    // SmartDashboard.putNumber("Pitch?", arr[4]);
    // SmartDashboard.putNumber("Yaw?", arr[5]);
  }
}