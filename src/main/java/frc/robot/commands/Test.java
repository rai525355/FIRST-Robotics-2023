// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;



import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ConeIntake;
import frc.robot.subsystems.CubeIntake;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.PoseLimelight;

public class Test extends CommandBase {
  /** Creates a new Test. */
  DrivetrainSubsystem drivetrain;
  PoseLimelight limelight;
  ConeIntake coneIntake;
  CubeIntake cubeIntake;
  Elevator elevator;
  AHRS navx;
  Timer timer = new Timer();
  boolean balanced = false;
  public Test(DrivetrainSubsystem d, PoseLimelight l, AHRS n, ConeIntake co, CubeIntake cu, Elevator e) {
    drivetrain = d;
    //limelight = l;
    navx = n;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    //addRequirements(limelight);
    // addRequirements(navx);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // timer.start();
    //drivetrain.zeroGyroscope();
    //new LineUpForCube(limelight, drivetrain);
    new CubeShooterHigh(coneIntake, cubeIntake);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
    // if(!timer.hasElapsed(2)) {
    //   drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
    //     1,
    //     0,
    //     0,
    //     drivetrain.getGyroscopeRotation()));
    // }
    // else {
    //   finished = true;
    // } 
    //drivetrain.setToZero();
    if (drivetrain.getPitch() < 1  && drivetrain.getPitch() > -1 && !balanced) {
      drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
        1,
        0,
        0,
        drivetrain.getGyroscopeRotation()));
    }
    else {
      // new AutonBalance(navx, drivetrain);
      balanced = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
      0,
      0,
      0,
      drivetrain.getGyroscopeRotation())); 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return balanced;
  }
}
