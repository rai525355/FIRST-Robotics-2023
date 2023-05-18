// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NewCamera extends SubsystemBase {
  /** Creates a new NewCamer. */
  public NewCamera() {}
  public VideoSource cam1;
  // public VideoSource cam2;
  public MjpegServer mjpeg1;
  // public MjpegServer mjpeg2;

  public void startCamera(){
    cam1 = CameraServer.startAutomaticCapture(0);
    // cam2 = CameraServer.startAutomaticCapture(1);
    //mjpeg1 = CameraServer.startAutomaticCapture(cam1);
    // mjpeg2 = CameraServer.startAutomaticCapture(cam2);
    ///mjpeg1.setFPS(15);
    //mjpeg1.setResolution(480, 360);
    // mjpeg2.setFPS(15);
    // mjpeg2.setResolution(480, 360);
    //SmartDashboard.putString("cameraServer1", mjpeg1.getListenAddress());
    // SmartDashboard.putString("cameraServer2", mjpeg2.getListenAddress());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}
