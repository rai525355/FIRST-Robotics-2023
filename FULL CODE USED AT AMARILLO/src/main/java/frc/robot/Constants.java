// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    
    //The left-to-right distance between the drivetrain wheels
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.508;
   
    //The front-to-back distance between the drivetrain wheels.
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.508;

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 2;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 1;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 21;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(23.37 + 334 + 9.93 + 69.34 + 354.83 + 2.636 + 357.8 + 358.94 + 3.16 + .79 - 3.5 + 1.5); // FIXME Measure and set front left steer offset

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 4;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 3;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 22;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(297.949219 + 12.216797 + 113.9 + .527344 + 1.14 + 1.23 + 180 + 358.85 + 1.14 + 356 + 2 + 1.6); // FIXME Measure and set front right steer offset

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 6;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 5;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 23;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(174.75 + 354.726 + .878906 + 4.04 + .791 + 359.82 + 1.8 - 2.4); // FIXME Measure and set back left steer offset

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 8;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 7;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 24;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(221.748047 + 233.998047 + .6 + .878906 + 359.82 + 2.812 + 358.68 + 2.8 - 4.5); // FIXME Measure and set back right steer offset

    public static final int coneIntakeCanID = 9;
    public static final double coneIntakeKp = 0;
    public static final double coneIntakeKi = 0;
    public static final double coneIntakeKd = 0;
    public static final double coneIntakeKf = 0;

    public static final int cubeIntakeCanID = 10;
    public static final double cubeIntakeKp = 0;
    public static final double cubeIntakeKi = 0;
    public static final double cubeIntakeKd = 0;
    public static final double cubeIntakeKf = 0;

    public static final int elevatorCanID = 11;

    // 03/25/starting constants
    // public static final double ElevatorKp = .3;
    // public static final double ElevatorKi = 0;
    // public static final double ElevatorKd = .05;
    // public static final double ElevatorKf = 0;

    public static final double ElevatorKp = .05;
    public static final double ElevatorKi = 0;
    public static final double ElevatorKd = .00;
    public static final double ElevatorKf = 0.0;

    public static final double ElevatorVelLimit = 10000;
    public static final double ElevatorAccelLimit = 32000;
    public static final int ElevatorSCurveStrength = 0;
    
    // public static final double elevatorDefaultPos = 0;
    // public static final double elevatorHighPos = 0;
    // public static final double elevatorPickUpPos = 0;









    
    
    //secondary controller
    public static final int zeroGyroButtonID = 9;
    public static final int coneInButtonID = 5;
    public static final int cubeInButtonID = 6;
    public static final int coneOutButtonID = 7;
    public static final int cubeShooterHighButtonID = 4;
    public static final int cubeShooterMidButtonID = 2;
    public static final int cubeShooterLowButtonID = 1;
    public static final int elevatorHighButtonID = 4;
    public static final int elevatorLowButtonID = 2;
    public static final int elevatorBottomButtonID = 1;
    public static final int cubeOut = 3;


    /* Testing buttons
    //public static final int cubeOutButtonID = 2;
    public static final int autoLLtoDriverID = 8;
    public static final int retroLLtoDriverID = 9;
    public static final int retroLLtoVisionID = 10;
    public static final int retroLLtoRetroID = 11;
    public static final int retroLLtoAprilID = 12;

    
    */
    public static final int elevatorUpID = 8;
    public static final int elevatorDownID = 8;
}
