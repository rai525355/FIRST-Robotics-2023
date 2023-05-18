// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class LineUp1 extends CommandBase {
  DrivetrainSubsystem drivetrain;

  boolean finished = false;

  Timer timer = new Timer();


  /** Creates a new AutoStarting1. */
  public LineUp1(DrivetrainSubsystem d) {
    drivetrain = d;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.zeroGyroscope();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!timer.hasElapsed(.55)) {
      drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, -2, 0, drivetrain.getGyroscopeRotation()));
    }
    if(timer.hasElapsed(.6) && drivetrain.getPitch() < 10) {
      drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(2, 0, 0, drivetrain.getGyroscopeRotation()));
    }
    else {
      finished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, drivetrain.getGyroscopeRotation()));

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
