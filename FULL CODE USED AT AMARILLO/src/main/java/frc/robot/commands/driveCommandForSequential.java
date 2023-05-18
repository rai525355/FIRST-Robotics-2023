// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class driveCommandForSequential extends CommandBase {
  private DrivetrainSubsystem drivetrain;
  private double x;
  private double y;
  private double r;
  private double time;
  private Timer timer = new Timer();

  /** Creates a new driveCommandForSequential. */
  public driveCommandForSequential(DrivetrainSubsystem d, double xGiven, double yGiven, double rGiven, double timeGiven) {
    // Use addRequirements() here to declare subsystem dependencies.
    drivetrain =d;
    x = xGiven;
    y = yGiven;
    r =rGiven;
    time = timeGiven;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!timer.hasElapsed(time)){
      drivetrain.simpleDrive(x, y, r);
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
    if(timer.hasElapsed(time)){
      return true;
    }
    else{
      return false;
    }
  }
}
