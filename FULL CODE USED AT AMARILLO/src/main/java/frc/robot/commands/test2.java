// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class test2 extends CommandBase {
  DrivetrainSubsystem drivetrain;
  Timer timer = new Timer();
  boolean finished = false;
  /** Creates a new test2. */
  public test2(DrivetrainSubsystem d) {
    drivetrain = d;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.zeroGyroscope();
    timer.start();
    // drivetrain.setToZero();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!timer.hasElapsed(1.7)) {
      drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
        1.5,
        0,
        0,
        drivetrain.getGyroscopeRotation()));
    }
    else {

    if ( drivetrain.getPitch() > 3) {//test with and in first if statement {
      //finished = true; // print "running" to check if this if statement is ever working
      drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
      1.5,
      0,
      0,
      drivetrain.getGyroscopeRotation()));
      // if() { //!timer.hasElapsed(2) && 
      //   finished = true;
       }
      else {
        finished = true;
      }
    
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
        0,
        0,
        0,
        drivetrain.getGyroscopeRotation()));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
