// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.RetroLimelight;

public class LineUpForCone extends CommandBase {
  RetroLimelight limelight;
  DrivetrainSubsystem drivetrain;
  boolean finished = false;
  /** Creates a new LineUpForCone. */
  public LineUpForCone(DrivetrainSubsystem d, RetroLimelight l) {
    limelight = l;
    drivetrain = d;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(limelight);
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(limelight.getX() > .1) {
      drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
        0,
        -1,
        0,
        drivetrain.getGyroscopeRotation()));
    }
    else if(limelight.getX() < -.1) {
      drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
        0,
        1,
        0,
        drivetrain.getGyroscopeRotation()));
    }
    else {
      finished = true;
        drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
          0,
          0,
          0,
          drivetrain.getGyroscopeRotation()));
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
