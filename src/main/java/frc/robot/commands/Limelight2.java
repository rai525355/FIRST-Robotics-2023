// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
 import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PoseLimelight;
import edu.wpi.first.wpilibj.Timer;


public class Limelight2 extends CommandBase {
     private double x;
     private double y;
     private double z;
     private double[] bbb = {1,2,3,4,5,6};

    private boolean working = false;
    private boolean working2 = false;

    DrivetrainSubsystem m_DrivetrainSubsystem;
    PoseLimelight m_Limelight;
    Timer timer = new Timer();

  /** Creates a new Limelight2. */
  public Limelight2(DrivetrainSubsystem drivetrain, PoseLimelight limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_DrivetrainSubsystem = drivetrain;
    m_Limelight = limelight;
    addRequirements(drivetrain);
    addRequirements(limelight);
  }
  public double maxMetersPerSec = 5.4864;

  public void displacementDrive(double xMeters, double yMeters){
    double hypot = Math.sqrt(((xMeters * xMeters) + (yMeters * yMeters)));
    double time = hypot/(maxMetersPerSec/4);
    double currTime = timer.get();
    time = time + currTime;
    while(!timer.hasElapsed(time)){
      m_DrivetrainSubsystem.drive(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                          xMeters / (maxMetersPerSec / 4),
                          yMeters / (maxMetersPerSec / 4),
                          0,
                          m_DrivetrainSubsystem.getGyroscopeRotation()
                  )
      );
    }
    m_DrivetrainSubsystem.simpleDrive(.1, 0, 0);
  }
  //Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     double[] arr = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_targetspace").getDoubleArray(bbb);
      
       x = arr[0];
       y = arr[1];
       z = arr[2];
  
       SmartDashboard.putNumber("Limelight X", x);
       SmartDashboard.putNumber("Limelight Y", y);
       SmartDashboard.putNumber("Limelight Z", z);

      // if (x > .3) {
      //   m_DrivetrainSubsystem.simpleDrive(1, 0, 0);
      // }
      // else if (x < .3) {
      //   m_DrivetrainSubsystem.simpleDrive(-1, 0, 0);
      // }
      // else {
         m_DrivetrainSubsystem.simpleDrive(0, 0, .5);
      // }
      //if (x > .1) {

        SmartDashboard.putBoolean("working", working);
        SmartDashboard.putBoolean("working2", working2);

        //SmartDashboard.putNumber("getX", m_Limelight.getX());
        working = true;
        //displacementDrive(.1,.1);
        //m_DrivetrainSubsystem.simpleDrive(0, 0, .1);
        //if (m_Limelight.getZ() < -.5) {
          //m_DrivetrainSubsystem.simpleDrive(0, 0, .1);
          working2 = true;
        }
        
        // SmartDashboard.putBoolean("working", working);
        // SmartDashboard.putBoolean("working2", working2);
        
      //}
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}