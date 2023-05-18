// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ConeIntake;
import frc.robot.subsystems.CubeIntake;
// import frc.robot.subsystems.Elevator;

public class CubeShooterHigh extends CommandBase {
  /** Creates a new CubeShooterHigh. */
  private ConeIntake cone;
  private CubeIntake cube;
  // private Elevator elevator;
  Timer timer = new Timer();
  
  public CubeShooterHigh(ConeIntake co, CubeIntake cu /*, Elevator e*/) {
    cone = co;
    cube = cu;
    // elevator = e;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(cone);
    addRequirements(cube);
    // addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // timer.reset();
    // timer.start();

    //  elevator.setPosition(20000);

    // if(timer.hasElapsed(.1)) {
    //   elevator.setPosition(0);
    // }
    

    //new ElevatorBottom(elevator);
    //elevator.setPosition(0);
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // System.out.println(timer.get());
    // if(timer.hasElapsed(1)) {
    //     // elevator.setPosition(0);
        
    //   }
      // if(timer.hasElapsed(.95)) {
        
      // }
      cone.shoot(0.99);
      cube.shoot(-1);
      
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    cone.stop();
    cube.stop();
    System.out.println(interrupted);
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
