// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.List;
//import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;

// import com.pathplanner.lib.PathConstraints;
// import com.pathplanner.lib.PathPlanner;
// import com.pathplanner.lib.PathPlannerTrajectory;
// import com.pathplanner.lib.auto.PIDConstants;
// import com.pathplanner.lib.auto.SwerveAutoBuilder;

//import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutonBalance;
import frc.robot.commands.AutonBalanceOne;
//import frc.robot.commands.AutonomousCube;
import frc.robot.commands.ConeIntakeIn;
import frc.robot.commands.ConeIntakeOut;
import frc.robot.commands.CubeIntakeIn;
import frc.robot.commands.CubeIntakeOut;
import frc.robot.commands.CubeShooterHigh;
import frc.robot.commands.CubeShooterLow;
import frc.robot.commands.CubeShooterMid;
import frc.robot.commands.CubeThenBalance;
//import frc.robot.commands.CubeIntakeOut;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DriveCommandWithZeroing;
import frc.robot.commands.ElevatorBottom;
import frc.robot.commands.ElevatorDown;
import frc.robot.commands.ElevatorLevelOne;
import frc.robot.commands.ElevatorLevelTwo;
import frc.robot.commands.ElevatorPickup;
import frc.robot.commands.ElevatorUp;
import frc.robot.commands.LineUpForCone;
import frc.robot.commands.LineUpForCube;
import frc.robot.commands.Test;
//import frc.robot.commands.ElevatorHigh;
//import frc.robot.commands.Limelight2;
//import frc.robot.commands.LimelightCommand;
import frc.robot.commands.autoBalance;
// import frc.robot.commands.autoPath;
import frc.robot.commands.autonomousIntake;
// import frc.robot.commands.coneThenBalance;
import frc.robot.commands.test2;
import frc.robot.commands.auton.AutoBalance1;
import frc.robot.commands.auton.AutoBalance2;
import frc.robot.commands.auton.AutoBalance3;
import frc.robot.commands.auton.AutoBalance6;
import frc.robot.commands.auton.AutoBalance7;
import frc.robot.commands.auton.AutoBalance8;
import frc.robot.commands.auton.AutoStarting1Group;
import frc.robot.commands.auton.AutoStarting3Command;
import frc.robot.commands.auton.AutoStarting3Group;
import frc.robot.commands.auton.AutoStarting6Group;
// import frc.robot.commands.auton.AutoStarting6Group;
// import frc.robot.commands.auton.AutoStarting8Group;
import frc.robot.subsystems.ConeIntake;
import frc.robot.subsystems.CubeIntake;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.NewCamera;
import frc.robot.subsystems.PoseLimelight;
import frc.robot.subsystems.RetroLimelight;

import static frc.robot.Constants.*;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer implements Supplier<Boolean>{

  // The robot's subsystems and commands are defined here...
  private final Joystick m_controller1 = new Joystick(0);
  private final Joystick m_controller2 = new Joystick(1);

  public static final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final Elevator m_elevator = new Elevator();
  //private final PoseLimelight m_Limelight = new PoseLimelight();
  private final ConeIntake m_coneIntake = new ConeIntake();
  private final CubeIntake m_cubeIntake = new CubeIntake();
  public final static PoseLimelight m_poseLimelight = new PoseLimelight();
  public final static RetroLimelight m_retroLimelight = new RetroLimelight();
  // public final Camera camera = new Camera();
  // public int count = 0;

  public NewCamera camera = new NewCamera();
  

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // if(count == 0){
    // camera.videoToDashboard();
    // count++;
    // }
    // else{
      camera.startCamera();
      
    // }

    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation

    // m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
    //     m_drivetrainSubsystem,
    //     () -> -modifyAxis(m_controller1.getRawAxis(1)) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
    //     () -> -modifyAxis(m_controller1.getRawAxis(0)) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
    //     () -> -modifyAxis(m_controller1.getRawAxis(4)) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    // ));

    m_drivetrainSubsystem.setDefaultCommand(new DriveCommandWithZeroing(
      m_drivetrainSubsystem, 
      () -> -modifyAxis(m_controller1.getRawAxis(1)) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
      () -> -modifyAxis(m_controller1.getRawAxis(0)) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
      () -> -modifyAxis(m_controller1.getRawAxis(4)) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));

      
    // m_drivetrainSubsystem.setToZero();

    
    
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // Trigger lineUp = new JoystickButton(m_controller1, 5);
    // lineUp.onTrue(new LineUpForCone(m_drivetrainSubsystem, m_retroLimelight));

    //button zeros the gyroscope
    Trigger zeroButton = new JoystickButton(m_controller2, 9);
    zeroButton.whileTrue(new RunCommand(() -> m_drivetrainSubsystem.zeroGyroscope(), m_drivetrainSubsystem));

    Trigger setToZero = new JoystickButton(m_controller1, 4);
    setToZero.whileTrue(new RunCommand(() -> m_drivetrainSubsystem.setToZero(), m_drivetrainSubsystem));

    // Trigger DriverCam = new JoystickButton(m_controller1, 8);
    // DriverCam.whileTrue(new RunCommand(() -> m_retroLimelight.changeMode(1), m_retroLimelight));

    Trigger cone_IN_button = new JoystickButton(m_controller2, coneInButtonID);
    cone_IN_button.whileTrue(new ConeIntakeIn(m_coneIntake));

    Trigger cone_OUT_button = new JoystickButton(m_controller2, coneOutButtonID);
    cone_OUT_button.whileTrue(new ConeIntakeOut(m_coneIntake));
    while (m_controller2.getRawAxis(2) > 0) {
      new ConeIntakeOut(m_coneIntake);
    }

    Trigger cube_IN_button = new JoystickButton(m_controller2, cubeInButtonID);
    cube_IN_button.whileTrue(new CubeIntakeIn(m_cubeIntake));

    //Cube backwards out of intake
    Trigger cube_BACK_button = new JoystickButton(m_controller2, 3);
    cube_BACK_button.whileTrue(new CubeIntakeOut(m_cubeIntake));

    Trigger cube_Shooter_High_button = new JoystickButton(m_controller2, cubeShooterHighButtonID);
    cube_Shooter_High_button.whileTrue(new CubeShooterHigh(m_coneIntake, m_cubeIntake));

    Trigger cube_Shooter_Mid_button = new JoystickButton(m_controller2, cubeShooterMidButtonID);
    cube_Shooter_Mid_button.whileTrue(new CubeShooterMid(m_coneIntake, m_cubeIntake));

    POVButton up = new POVButton(m_controller2, 0);
    up.onTrue(new ElevatorLevelTwo(m_elevator));

    POVButton right = new POVButton(m_controller2, 90);
    right.onTrue(new ElevatorLevelOne(m_elevator));

    POVButton down = new POVButton(m_controller2, 180);
    down.onTrue(new ElevatorBottom(m_elevator));

    POVButton left = new POVButton(m_controller2, 270);
    left.onTrue(new ElevatorPickup(m_elevator));
    left.onFalse(new ElevatorLevelOne(m_elevator));

    Trigger cube_Shooter_Low_button = new JoystickButton(m_controller2, cubeShooterLowButtonID);
    cube_Shooter_Low_button.whileTrue(new CubeShooterLow(m_coneIntake, m_cubeIntake, m_elevator));



    // Trigger elevator_High_button = new JoystickButton(m_controller1, elevatorHighButtonID);
    // elevator_High_button.onTrue(new ElevatorLevelTwo(m_elevator));

    // Trigger elevator_Low_button = new JoystickButton(m_controller1, elevatorLowButtonID);
    // elevator_Low_button.onTrue(new ElevatorLevelOne(m_elevator));

    // Trigger elevator_Bottom_button = new JoystickButton(m_controller1, elevatorBottomButtonID);
    // elevator_Bottom_button.onTrue(new ElevatorBottom(m_elevator));

    // Trigger elevator_Pickup_button = new JoystickButton(m_controller1, elevatorPickupButtonID);
    // elevator_Pickup_button.onTrue(new ElevatorPickup(m_elevator));

    // Trigger auto = new JoystickButton(m_controller2, 9);
    // auto.onTrue(new Test(m_drivetrainSubsystem, m_poseLimelight, m_navx, m_coneIntake, m_cubeIntake, m_elevator));
    

    



    //Testing Buttons

    // Trigger elevator_UP_button = new JoystickButton(m_controller2, elevatorUpID);
    // elevator_UP_button.whileTrue(new ElevatorUp(m_elevator));

    // Trigger elevator_DOWN_button = new JoystickButton(m_controller2, elevatorDownID);
    // elevator_DOWN_button.whileTrue(new ElevatorDown(m_elevator));

    // Trigger set_Auto_LL_to_DriverCam = new JoystickButton(m_controller2, autoLLtoDriverID);
    // set_Auto_LL_to_DriverCam.onTrue(new RunCommand(() -> m_poseLimelight.changeMode(1), m_poseLimelight));

    // Trigger set_Retro_LL_to_DriverCam = new JoystickButton(m_controller2, retroLLtoDriverID);
    // set_Retro_LL_to_DriverCam.onTrue(new RunCommand(() -> m_retroLimelight.changeMode(1), m_retroLimelight));

    // Trigger set_Retro_LL_to_VisionCam = new JoystickButton(m_controller2, retroLLtoVisionID);
    // set_Retro_LL_to_VisionCam.onTrue(new RunCommand(() -> m_retroLimelight.changeMode(0), m_retroLimelight));

    // Trigger set_Retro_LL_to_Retro = new JoystickButton(m_controller2, retroLLtoRetroID);
    // set_Retro_LL_to_Retro.onTrue(new RunCommand(() -> m_retroLimelight.setPipeline(0), m_retroLimelight));

    // Trigger set_Retro_LL_to_April = new JoystickButton(m_controller2, retroLLtoAprilID);
    // set_Retro_LL_to_April.onTrue(new RunCommand(() -> m_retroLimelight.setPipeline(1), m_retroLimelight));


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    return new CubeThenBalance(m_drivetrainSubsystem, m_cubeIntake, m_coneIntake);
    // return new AutoStarting1Group(m_drivetrainSubsystem, m_coneIntake, m_cubeIntake);


    //      PATHPLANNER
    // HashMap<String, Command> eventMap = new HashMap<String, Command>();

    // // eventMap.put("shooter", new CubeShooterHigh(m_coneIntake, m_cubeIntake, m_elevator));
    // // eventMap.put("intake1", new autonomousIntake(m_cubeIntake));
    // // eventMap.put("shooter1", new CubeShooterHigh(m_coneIntake, m_cubeIntake, m_elevator));
    // // eventMap.put("intake2", new autonomousIntake(m_cubeIntake));
    // // eventMap.put("shooter2", new CubeShooterHigh(m_coneIntake, m_cubeIntake, m_elevator));
    // // eventMap.put("intake3", new autonomousIntake(m_cubeIntake));
    // // eventMap.put("shooter3", new CubeShooterHigh(m_coneIntake, m_cubeIntake, m_elevator));
    // // eventMap.put("intake4", new autonomousIntake(m_cubeIntake));

    // SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
    //   m_drivetrainSubsystem::getPose,
    //   m_drivetrainSubsystem::resetOdometry,
    //   m_drivetrainSubsystem.m_kinematics, 
    //   new PIDConstants(0.25, 0.0 ,0.3),
    //   new PIDConstants(0.25, 0.0, 0.2), 
    //   m_drivetrainSubsystem::setModuleStates,
    //   eventMap,
    //   true,
    //   m_drivetrainSubsystem
    // );
    // m_drivetrainSubsystem.zeroGyroscope();
    // List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("New New New Path", new PathConstraints(2, 3));
    // return autoBuilder.fullAuto(pathGroup);
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.025);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }

  //Here to make code gods happy
  @Override
  public Boolean get() {
    return null;
  }
}
