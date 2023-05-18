// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ConeIntake;
import frc.robot.subsystems.CubeIntake;
import frc.robot.subsystems.Elevator;

public class CubeShooterMid extends CommandBase {
  /** Creates a new CubeShooterHigh. */
  private ConeIntake cone;
  private CubeIntake cube;
  Timer timer = new Timer();
  
  public CubeShooterMid(ConeIntake co, CubeIntake cu) {
    cone = co;
    cube = cu;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(cone);
    addRequirements(cube);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //new ElevatorBottom(elevator);
    // elevator.setPosition(40000);
    // new ElevatorBottom(elevator);
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    cone.shoot(.9);
    cube.shoot(-.9);
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
    if(timer.hasElapsed(.5)){
      return true;
    } 
    else {
      return false;
    }
  }
}
