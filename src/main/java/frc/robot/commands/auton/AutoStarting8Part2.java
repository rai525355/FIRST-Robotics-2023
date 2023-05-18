// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoStarting8Part2 extends CommandBase {
  DrivetrainSubsystem drivetrain;
  Timer xTimer = new Timer();
  
  /** Creates a new AutoStarting8Part2. */
  public AutoStarting8Part2(DrivetrainSubsystem d) {
    drivetrain = d;
    // Use addRequirements() here to declare subsystem dependencies.a
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!xTimer.hasElapsed(3)) {
      drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(-1, 0, 0, drivetrain.getGyroscopeRotation()));
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
    return false;
  }
}
