// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PoseLimelight;
import frc.robot.subsystems.DrivetrainSubsystem;

public class LineUpForCube extends CommandBase {
  PoseLimelight limelight = new PoseLimelight();
  DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();
  boolean finished = false;
  /** Creates a new lineUpForCube. */
  public LineUpForCube(PoseLimelight l, DrivetrainSubsystem d) {
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
    if (limelight.getTx() > .1) {
      //go left
      //drivetrain.simpleDrive(-0.5, 0 ,0);
      drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
        1,
        0,
        0,
        drivetrain.getGyroscopeRotation()));
    }
    
    if (limelight.getTx() < -0.1) {
      //go right
      //drivetrain.simpleDrive(0.5, 0 ,0);
      drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
        0,
        0,
        0,
        drivetrain.getGyroscopeRotation()));
    }

    if (limelight.getTx() < 0.1 && limelight.getTx() > -0.1) {
      //drivetrain.simpleDrive(0, 0 ,0);
      finished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //drivetrain.simpleDrive(0,0,0);
    drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
        1,
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