// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class ElevatorLevelOne extends CommandBase {
  Elevator elevator;
  double target = 60000;
  /** Creates a new ElevatorPickup. */
  public ElevatorLevelOne(Elevator e) {
    elevator = e;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.setPosition(target);
    //System.out.println("initialize");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //System.out.println("execute");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.stop();
    //System.out.println("done");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return Math.abs(elevator.getMotor().getSelectedSensorPosition() - target) < 1000;
    return false;
  }
}
