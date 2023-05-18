package frc.robot.commands;

//import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.math.geometry.Translation2d;
//import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class autoBalance extends CommandBase {
    private final AHRS navx;
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final double platformAngle = 15;
    private static final double BALANCE_THRESHOLD = 3.0; // degrees
    private boolean balanced;
    Timer timer = new Timer();


    public autoBalance(AHRS navx, DrivetrainSubsystem m_drivetrainSubsystem) {
        this.navx = navx;
        this.m_drivetrainSubsystem = m_drivetrainSubsystem;
        //this.platformAngle = navx.getPitch();
        addRequirements(m_drivetrainSubsystem);
    }

    @Override
    public void initialize() {
      // m_drivetrainSubsystem.simpleDrive(.5, .5, 0);
      // if(!timer.hasElapsed(2)) {
      //   m_drivetrainSubsystem.simpleDrive(0, -1, 0);
      // }
      // new driveCommandForSequential(m_drivetrainSubsystem, 0, -1, 0, 2);
      
      
    }

    @Override
    public void execute() {

      // if(navx.getPitch() < .5 && !balanced) {
        
      // }
    

        // Calculate the difference between the angle of the robot and the angle of the platform
        double angleDiff = platformAngle - navx.getPitch();

        // If the robot is tilted beyond the balance threshold, adjust the swerve modules to move it towards the lower side of the platform
        if (Math.abs(navx.getPitch()) > BALANCE_THRESHOLD) {

            double speed = Math.max(0.1, Math.min(1.0, Math.abs(angleDiff) / 10.0)) * 10;
            if (navx.getPitch() < (BALANCE_THRESHOLD + 2) && navx.getPitch() > (-BALANCE_THRESHOLD - 2)) {
              speed /= 2;
            }

            if (navx.getPitch() > 1) {
              m_drivetrainSubsystem.simpleDrive(speed, 0, 0);
            } 
            else if (navx.getPitch() < -1) {
              m_drivetrainSubsystem.simpleDrive(-speed, 0, 0);
            }
        } else {
          m_drivetrainSubsystem.simpleDrive(0,0,0);
          balanced =  true;
        }
    }

    @Override
    public void end(boolean interrupted) {
      m_drivetrainSubsystem.simpleDrive(0, 0, 0);;
    }

    @Override
    public boolean isFinished() {
      return balanced;
    }
}
