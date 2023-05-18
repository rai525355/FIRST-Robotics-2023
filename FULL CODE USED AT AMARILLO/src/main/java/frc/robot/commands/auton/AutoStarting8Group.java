// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.CubeShooterHigh;
import frc.robot.commands.CubeShooterMid;
import frc.robot.commands.autonomousIntake;
import frc.robot.subsystems.ConeIntake;
import frc.robot.subsystems.CubeIntake;
import frc.robot.subsystems.DrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoStarting8Group extends SequentialCommandGroup {
  /** Creates a new AutoStarting1Group. */
  public AutoStarting8Group(DrivetrainSubsystem drivetrain, ConeIntake cone, CubeIntake cube) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new CubeShooterMid(cone, cube));
    addCommands(new AutoStarting8Command(drivetrain, cube, cone));
    // addCommands(new autonomousIntake(cube));
    // addCommands(new AutoStarting8Part2(drivetrain));
  }
}
