// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.commands.CubeShooterHigh;
import frc.robot.commands.autonomousIntake;
import frc.robot.subsystems.ConeIntake;
import frc.robot.subsystems.CubeIntake;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoStarting1Command extends CommandBase {
  DrivetrainSubsystem drivetrain;

  boolean intakeDone = false;
  boolean finished = false;

  Timer xTimer = new Timer();
  Timer yTimer = new Timer();

  /** Creates a new AutoStarting1. */
  public AutoStarting1Command(DrivetrainSubsystem d) {
    drivetrain = d;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.zeroGyroscope();
    //new CubeShooterHigh(cone, cube);
    yTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!yTimer.hasElapsed(.2)) {
      drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 1, 0, drivetrain.getGyroscopeRotation()));
    }
    else {
      xTimer.start();
    }
    if(!xTimer.hasElapsed(4.5)) {
      drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(1, 0, 0, drivetrain.getGyroscopeRotation()));
    }
    else {
      finished = true;
    }
    // if(xTimer.hasElapsed(1.5) && !intakeDone) {
    //   // new autonomousIntake(cube);
    //   intakeDone = true;
    // }
    // if(xTimer.hasElapsed(4.5) && !xTimer.hasElapsed(7.5)) {
    //   drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(-1, 0, 0, drivetrain.getGyroscopeRotation()));
    // }
    // else {
    //   finished = false;
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, drivetrain.getGyroscopeRotation()));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return false;
    return finished;
  }
}
