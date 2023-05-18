// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveCommandWithZeroing extends SequentialCommandGroup {
  DrivetrainSubsystem drivetrain;
  /** Creates a new DriveCommandWithZeroing. */
  public DriveCommandWithZeroing(DrivetrainSubsystem d, 
                                DoubleSupplier translationXSupplier,
                                DoubleSupplier translationYSupplier,
                                DoubleSupplier rotationSupplier) {
    drivetrain = d;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new SetToZero(drivetrain)); //Call set to zero then call default drive command
    addCommands(new DefaultDriveCommand(drivetrain, translationXSupplier, translationYSupplier, rotationSupplier));
  }
}
