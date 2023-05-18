// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class NewBalance extends CommandBase {
  DrivetrainSubsystem drivetrain;
  /** Creates a new NewBalance. */
  public NewBalance(DrivetrainSubsystem d) {
    drivetrain = d;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(drivetrain.getPitch() > 3){
      drivetrain.simpleDrive(1, 0, 0);
     }
     else if(drivetrain.getPitch() < 3){
      drivetrain.simpleDrive(-1, 0, 0);
     }
     else{
      drivetrain.simpleDrive(0, 0, 0);
     }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.simpleDrive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
