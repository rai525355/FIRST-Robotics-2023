// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ConeIntake;
import frc.robot.subsystems.CubeIntake;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.PoseLimelight;

public class AutonomousDock extends CommandBase {
  /** Creates a new Autonomous. */
  PoseLimelight limelight;
  DrivetrainSubsystem drivetrain;
  Elevator elevator;
  ConeIntake cone;
  CubeIntake cube;
  double x;
  double y;
  Timer timer = new Timer();
  boolean pos1 = false;
  boolean pos2 = false;
  boolean pos3 = false;
  //boolean pos4 = false;
  boolean finished = false;

  long startID;
  public AutonomousDock(PoseLimelight pl, DrivetrainSubsystem d, Elevator e, ConeIntake co, CubeIntake cu) {
    limelight = pl;
    drivetrain = d;
    elevator = e;
    cone = co;
    cube = cu;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(limelight);
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startID = PoseLimelight.getId();
    new CubeShooterHigh(cone, cube);
    timer.start();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    new ElevatorLevelTwo(elevator);
    //new driveCommandForSequential(drivetrain, 0, 0, .5, 1); // Check to see how much time to run at what speed
    new CubeShooterHigh(cone, cube);

    if(startID == 1 || startID == 2 || startID == 3) {
      if(limelight.getRedX() !=0 && limelight.getRedY() != 0){
        x = limelight.getRedX();
        y = limelight.getRedY();
      }
    }

    if(startID == 6 || startID == 7 || startID == 8) {
      if(limelight.getBlueX() !=0 && limelight.getBlueY() != 0){
        x = limelight.getBlueX();
        y = limelight.getBlueY();
      }

      if(startID == 1 || startID == 8 && !pos1) {
        if(y < 7.35) {
          drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 1, 0, drivetrain.getGyroscopeRotation()));
        }
        else {
          drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, drivetrain.getGyroscopeRotation()));
          pos1 = true;
          timer.restart();
        }
      }

      if(startID == 1 || startID == 8 && pos1 && !pos2) {
        if(!timer.hasElapsed(3.5)) {
          drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(1.5, 0, 0, drivetrain.getGyroscopeRotation()));
        }
        else {
          drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, drivetrain.getGyroscopeRotation()));
          pos2 = true;
          timer.restart();
        }
        if(timer.hasElapsed(2) && !timer.hasElapsed(3.5)) {
          new CubeIntakeIn(cube);
        }
      }

      if(startID == 1 || startID == 8 && pos2 && !pos3) {
        if(!timer.hasElapsed(2)) {
          drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(-1.5, -0.75, 0, drivetrain.getGyroscopeRotation()));
        }
        else {
          drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, drivetrain.getGyroscopeRotation()));
          pos3 = true;
          timer.restart();
        }
      }

      if(startID == 1 || startID == 8 && pos3 && !finished) {
        if(!timer.hasElapsed(1)) {
          drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(-1, 0, 0, drivetrain.getGyroscopeRotation()));
        }
        else {
          drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, drivetrain.getGyroscopeRotation()));
          //pos4 = true;
          finished = true;
        }
      }
    }

    if(startID == 2 || startID == 6 && !finished) {
      new test2(drivetrain);
      finished = true;
    }


    
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, drivetrain.getGyroscopeRotation()));
    /*if we have cool leds:
      if we balance show cool rainbow leds and if we don't show red lights  
    */
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
