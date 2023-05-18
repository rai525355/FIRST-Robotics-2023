// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.Mk4ModuleConfiguration;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;

import static frc.robot.Constants.*;

public class DrivetrainSubsystem extends SubsystemBase {
  /**
   * The maximum voltage that will be delivered to the drive motors.
   * <p>
   * This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
   */
  public static double MAX_VOLTAGE = 12.0; //default 12
   /**
   * The maximum velocity of the robot in meters per second.
   * <p>
   * This is a measure of how fast the robot should be able to drive in a straight line.
   */
  public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
          SdsModuleConfigurations.MK4I_L3.getDriveReduction() *
          SdsModuleConfigurations.MK4I_L3.getWheelDiameter() * Math.PI;
  /**
   * The maximum angular velocity of the robot in radians per second.
   * <p>
   * This is a measure of how fast the robot can rotate in place.
   */
  // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
          Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

  public final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
          // Front left
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Front right
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back left
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back right
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
  );

  // By default we use a Pigeon for our gyroscope. But if you use another gyroscope, like a NavX, you can change this.
  // The important thing about how you configure your gyroscope is that rotating the robot counter-clockwise should
  // cause the angle reading to increase until it wraps back over to zero.
public final AHRS m_navx = new AHRS(Port.kMXP); // NavX connected over MXP

  // These are our modules. We initialize them in the constructor.
  private final SwerveModule m_frontLeftModule;
  private final SwerveModule m_frontRightModule;
  private final SwerveModule m_backLeftModule;
  private final SwerveModule m_backRightModule;

  private final double DRIVE_CURRENT_LIMIT = 40.0;
  private final Mk4ModuleConfiguration CONFIGURATION;
  

  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  private CANCoder c1 = new CANCoder(21);
  private CANCoder c2 = new CANCoder(22);
  private CANCoder c3 = new CANCoder(23);
  private CANCoder c4 = new CANCoder(24);

  public PIDController driftCorrection_pid;
  public double desiredHeading;

  Timer timer = new Timer();

  private final SwerveDrivePoseEstimator m_poseEstimator;

  public DrivetrainSubsystem() {
    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
    m_navx.enableBoardlevelYawReset(true);
    m_navx.zeroYaw();
    driftCorrection_pid = getdriftCorrection_pid();
    

    CONFIGURATION = new Mk4ModuleConfiguration();
    CONFIGURATION.setDriveCurrentLimit(DRIVE_CURRENT_LIMIT);
    //CONFIGURATION.setSteerPID();
    

    m_frontLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
            tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(0, 0),
                    CONFIGURATION,
            Mk4iSwerveModuleHelper.GearRatio.L3,
            FRONT_LEFT_MODULE_DRIVE_MOTOR,
            FRONT_LEFT_MODULE_STEER_MOTOR,
            FRONT_LEFT_MODULE_STEER_ENCODER,
            FRONT_LEFT_MODULE_STEER_OFFSET
    );

    m_frontRightModule = Mk4iSwerveModuleHelper.createFalcon500(
            tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(2, 0),
                    CONFIGURATION,
            Mk4iSwerveModuleHelper.GearRatio.L3,
            FRONT_RIGHT_MODULE_DRIVE_MOTOR,
            FRONT_RIGHT_MODULE_STEER_MOTOR,
            FRONT_RIGHT_MODULE_STEER_ENCODER,
            FRONT_RIGHT_MODULE_STEER_OFFSET
    );

    m_backLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
            tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(4, 0),
                    CONFIGURATION,
            Mk4iSwerveModuleHelper.GearRatio.L3,
            BACK_LEFT_MODULE_DRIVE_MOTOR,
            BACK_LEFT_MODULE_STEER_MOTOR,
            BACK_LEFT_MODULE_STEER_ENCODER,
            BACK_LEFT_MODULE_STEER_OFFSET
    );

    m_backRightModule = Mk4iSwerveModuleHelper.createFalcon500(
            tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(6, 0),
                    CONFIGURATION,
            Mk4iSwerveModuleHelper.GearRatio.L3,
            BACK_RIGHT_MODULE_DRIVE_MOTOR,
            BACK_RIGHT_MODULE_STEER_MOTOR,
            BACK_RIGHT_MODULE_STEER_ENCODER,
            BACK_RIGHT_MODULE_STEER_OFFSET
    );

    setToZero();

    m_poseEstimator =
      new SwerveDrivePoseEstimator(
          m_kinematics,
          getGyroscopeRotation(),
          new SwerveModulePosition[] {
            new SwerveModulePosition(m_frontLeftModule.getDriveVelocity(), new Rotation2d(m_frontLeftModule.getSteerAngle() - FRONT_LEFT_MODULE_STEER_OFFSET)),
            new SwerveModulePosition(m_frontRightModule.getDriveVelocity(), new Rotation2d(m_frontRightModule.getSteerAngle() - FRONT_RIGHT_MODULE_STEER_OFFSET)),
            new SwerveModulePosition(m_backLeftModule.getDriveVelocity(), new Rotation2d(m_backLeftModule.getSteerAngle() - BACK_LEFT_MODULE_STEER_OFFSET)),
            new SwerveModulePosition(m_backRightModule.getDriveVelocity(), new Rotation2d(m_backRightModule.getSteerAngle() - BACK_RIGHT_MODULE_STEER_OFFSET))
          },
          new Pose2d(0, 0, new Rotation2d(0))); //
        //   VecBuilder.fill(0.85, 0.85, Units.degreesToRadians(0.5)), // initiial was 0.05 for both on top and 0.5 for bottom, 0.05, 0.05, 0.65
        //   VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(60)));
        
  }

  public boolean setToZero() {      
        m_frontLeftModule.set(0, 0);
        m_frontRightModule.set(0, 0);
        m_backLeftModule.set(0, 0);
        m_backRightModule.set(0, 0);
        System.out.println("setToZero");
        // if(m_frontLeftModule.getSteerAngle() < 1 && m_frontLeftModule.getSteerAngle() > -1
        // && m_frontRightModule.getSteerAngle() < 1 && m_frontLeftModule.getSteerAngle() > -1
        // && m_backLeftModule.getSteerAngle() < 1 && m_frontLeftModule.getSteerAngle() > -1
        // && m_backRightModule.getSteerAngle() < 1 && m_frontLeftModule.getSteerAngle() > -1) 
        // {
        //         return true;
        // }
        // else {
        //         return false;
        // }
        return true;
  }

  double pXY = 0;

  public void driftCorrection(ChassisSpeeds chassis) {
                double xy = Math.abs(chassis.vxMetersPerSecond) + Math.abs(chassis.vyMetersPerSecond);

                if (Math.abs(chassis.omegaRadiansPerSecond) > 0.0 || pXY <= 0) {
                        desiredHeading = getGyroscopeRotation().getDegrees();
                } else if (xy > 0) {
                        chassis.omegaRadiansPerSecond += driftCorrection_pid
                                        .calculate(getGyroscopeRotation().getDegrees(), desiredHeading);
                }
                pXY = xy;
  }

  /**
   * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
   * 'forwards' direction.
   */
  public void zeroGyroscope() {
        m_navx.zeroYaw();
  }

  public double getPitch() {
        return m_navx.getPitch() - 2.5;
  }

  public Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
  }

  public double getPoseHeading() {
        return m_poseEstimator.getEstimatedPosition().getRotation().getDegrees();
  }

  public void resetOdometry(Pose2d pose) {
        m_poseEstimator.resetPosition(
        getGyroscopeRotation(),
          new SwerveModulePosition[] {
            new SwerveModulePosition(m_frontLeftModule.getDriveVelocity(), new Rotation2d(m_frontLeftModule.getSteerAngle() - FRONT_LEFT_MODULE_STEER_OFFSET)),
            new SwerveModulePosition(m_frontRightModule.getDriveVelocity(), new Rotation2d(m_frontRightModule.getSteerAngle() - FRONT_RIGHT_MODULE_STEER_OFFSET)),
            new SwerveModulePosition(m_backLeftModule.getDriveVelocity(), new Rotation2d(m_backLeftModule.getSteerAngle() - BACK_LEFT_MODULE_STEER_OFFSET)),
            new SwerveModulePosition(m_backRightModule.getDriveVelocity(), new Rotation2d(m_backRightModule.getSteerAngle() - BACK_RIGHT_MODULE_STEER_OFFSET))
          },
          pose
        );
  }

  public void setModuleStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);
        m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
        m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
        m_backLeftModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());
        m_backRightModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
  }

  public Rotation2d getGyroscopeRotation() {
    
    if (m_navx.isMagnetometerCalibrated()) {
      // We will only get valid fused headings if the magnetometer is calibrated
      return Rotation2d.fromDegrees(m_navx.getFusedHeading());
    }
    // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes the angle increase.
    return Rotation2d.fromDegrees(360.0 - m_navx.getYaw());
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    m_chassisSpeeds = chassisSpeeds;
    //System.out.println(chassisSpeeds.omegaRadiansPerSecond);
  }

  public void simpleDrive(double x, double y, double r){
        m_chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        x,
        y,
        r,
        getGyroscopeRotation());
  }

  public double maxMetersPerSec = 5.4864;

  public void displacementDrive(double xMeters, double yMeters){
    double hypot = Math.sqrt(((xMeters * xMeters) + (yMeters * yMeters)));
    double time = hypot/(maxMetersPerSec/4);
    double currTime = timer.get();
    time = time + currTime;
    while(!timer.hasElapsed(time)){
      drive(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                          xMeters / (maxMetersPerSec / 4),
                          yMeters / (maxMetersPerSec / 4),
                          0,
                          getGyroscopeRotation()
                  )
      );
    }
      drive(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                          0,
                          0,
                          0,
                          getGyroscopeRotation()
                  )
      );
  }

  public void setMaxVoltage(double x) {
        MAX_VOLTAGE = x;
  }

//   public command        

  @Override
  public void periodic() {
    //driftCorrection(m_chassisSpeeds);

    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

    SmartDashboard.putNumber("CANCoder 1", c1.getPosition());
    SmartDashboard.putNumber("CANCoder 1 Absolute", c1.getAbsolutePosition());
    SmartDashboard.putNumber("CANCoder 2", c2.getPosition());
    SmartDashboard.putNumber("CANCoder 2 Absolute", c2.getAbsolutePosition());
    SmartDashboard.putNumber("CANCoder 3", c3.getPosition());
    SmartDashboard.putNumber("CANCoder 3 Absolute", c3.getAbsolutePosition());
    SmartDashboard.putNumber("CANCoder 4", c4.getPosition());
    SmartDashboard.putNumber("CANCoder 4 Absolute", c4.getAbsolutePosition());

    SmartDashboard.putNumber("NavX Yaw", m_navx.getYaw());
    SmartDashboard.putNumber("NavX Pitch", getPitch());

    SmartDashboard.putNumber("Max Voltage", MAX_VOLTAGE);
//     Joystick m_controller = new Joystick(2);
//     setMaxVoltage(Math.abs((-m_controller.getRawAxis(3)) + 1) * 6);
//     if (MAX_VOLTAGE < 1) {
//         MAX_VOLTAGE = 1;
//     }

    //Limelight m_Limelight = new Limelight();
    //SmartDashboard.putNumber("drivetrain getX", m_Limelight.getX());

    m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
    m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
    m_backLeftModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());
    m_backRightModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
  }

  private PIDController getdriftCorrection_pid() {
        return new PIDController(0, 0, 0); // p: 0.09, i: 0.0, d: 0.0003
  }
}
