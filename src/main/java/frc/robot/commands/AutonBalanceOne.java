// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;


import frc.robot.subsystems.DrivetrainSubsystem;

// import com.kauailabs.navx.frc.AHRS;



public class AutonBalanceOne extends CommandBase {
  // int count;
  // AHRS navx;
  // Timer timer = new Timer();
  boolean balanced = false;
  DrivetrainSubsystem drivetrainSubsystem;

  public AutonBalanceOne(DrivetrainSubsystem drivetrainSubsystem) {
    // navx = n;
    this.drivetrainSubsystem = drivetrainSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // timer.start();
    // count = 0;
    // drivetrainSubsystem.zeroGyroscope();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (drivetrainSubsystem.getPitch() > 5) {
      //drivetrainSubsystem.drive(new ChassisSpeeds(0.0, -.5, 0.0));
      drivetrainSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
        .5,
        0,
        0,
        drivetrainSubsystem.getGyroscopeRotation()));
    }

    if (drivetrainSubsystem.getPitch() < -5) {
      //drivetrainSubsystem.drive(new ChassisSpeeds(0.0, -.5, 0.0));
      drivetrainSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
        -.3,
        0,
        0,
        drivetrainSubsystem.getGyroscopeRotation()));
    }

    if (drivetrainSubsystem.getPitch() > -5 && drivetrainSubsystem.getPitch() < 5) {
      drivetrainSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
        0,
        0,
        0,
        drivetrainSubsystem.getGyroscopeRotation()));
    }
  }

  //   if (drivetrainSubsystem.getPitch() > 2 && drivetrainSubsystem.getPitch() < 5) {
  //     //drivetrainSubsystem.drive(new ChassisSpeeds(0.0, -.5, 0.0));
  //     drivetrainSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
  //       1,
  //       0,
  //       0,
  //       drivetrainSubsystem.getGyroscopeRotation()));
  //   }

  //   if (drivetrainSubsystem.getPitch() < -2 && drivetrainSubsystem.getPitch() > -5) {
  //     //drivetrainSubsystem.drive(new ChassisSpeeds(0.0, -.5, 0.0));
  //     drivetrainSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
  //       -1,
  //       0,
  //       0,
  //       drivetrainSubsystem.getGyroscopeRotation()));
  //   }

  //   // if (drivetrainSubsystem.getPitch() < 2 && drivetrainSubsystem.getPitch() > -2 && !balanced) {
  //   //   count++;
  //   //   timer.reset();
  //   //   timer.start();
  //   //   if(timer.hasElapsed(1) && count > 2){
  //   //     if(drivetrainSubsystem.getPitch() < 2 && drivetrainSubsystem.getPitch() > -2 && !balanced) {
  //   //       balanced = true;
  //   //     }
  //   //   }
  //   // }
  //   if (drivetrainSubsystem.getPitch() > -1 && drivetrainSubsystem.getPitch() < 1);
  //     // balanced = true;
  //     drivetrainSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0 ,drivetrainSubsystem.getGyroscopeRotation()));
  // }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  //   drivetrainSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
  //       0,
  //       0,
  //       0,
  //       drivetrainSubsystem.getGyroscopeRotation()));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
