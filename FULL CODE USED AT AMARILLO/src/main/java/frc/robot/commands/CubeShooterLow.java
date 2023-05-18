// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ConeIntake;
import frc.robot.subsystems.CubeIntake;
import frc.robot.subsystems.Elevator;

public class CubeShooterLow extends CommandBase {
  /** Creates a new CubeShooterHigh. */
  private ConeIntake cone;
  private CubeIntake cube;
  private Elevator elevator;
  
  public CubeShooterLow(ConeIntake co, CubeIntake cu, Elevator e) {
    cone = co;
    cube = cu;
    elevator = e;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(cone);
    addRequirements(cube);
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //new ElevatorBottom(elevator);
    //elevator.setPosition(40000);
    //new ElevatorBottom(elevator);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    cone.shoot(.5);
    cube.shoot(-.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    cone.stop();
    cube.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
