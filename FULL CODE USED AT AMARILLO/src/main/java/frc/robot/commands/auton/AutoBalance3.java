package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.CubeShooterHigh;
import frc.robot.commands.test2;
import frc.robot.subsystems.ConeIntake;
import frc.robot.subsystems.CubeIntake;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoBalance3 extends SequentialCommandGroup {
    
    DrivetrainSubsystem mDrivetrain;
    CubeIntake mCubeIntake;
    ConeIntake mConeIntake;

    public AutoBalance3(DrivetrainSubsystem drivetrain, CubeIntake cubeIntake, ConeIntake coneIntake) {
        mDrivetrain = drivetrain;
        mCubeIntake = cubeIntake;
        mConeIntake = coneIntake;

        addCommands(new CubeShooterHigh(coneIntake, cubeIntake));
        addCommands(new LineUp3(mDrivetrain));
        addCommands(new test2(mDrivetrain));        
    }

}
