// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  
  
  /** Creates a new Elevator. */
  WPI_TalonFX motor;
  TalonFXConfiguration config = new TalonFXConfiguration();
  

  public Elevator() {
    motor = new WPI_TalonFX(elevatorCanID);
    motor.configFactoryDefault();
    motor.setNeutralMode(NeutralMode.Brake);
    motor.setSelectedSensorPosition(0);
    //motor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    //StatorCurrentLimitConfiguration limitConfiguration = new StatorCurrentLimitConfiguration(false, BACK_LEFT_MODULE_STEER_MOTOR, BACK_LEFT_MODULE_STEER_ENCODER, BACK_LEFT_MODULE_DRIVE_MOTOR)
    
    // StatorCurrentLimitConfiguration limitConfig = new StatorCurrentLimitConfiguration(false, Constants.currentLimit, 
    //     Constants.ThresholdCurrent, Constants.ThresholdTime);
    // leftMotor.configStatorCurrentLimit(limitConfig);
    //motor.set
    config.slot0.kP = ElevatorKp;
    config.slot0.kI = ElevatorKi;
    config.slot0.kD = ElevatorKd;
    config.slot0.kF = ElevatorKf;

    config.slot0.closedLoopPeakOutput = 0.8;

    config.closedloopRamp = 0.5;
    
    config.motionCruiseVelocity = ElevatorVelLimit;
    config.motionAcceleration = ElevatorAccelLimit;
    config.motionCurveStrength = ElevatorSCurveStrength;

    // motor.config_kF(0, ElevatorKf);
    // motor.config_kP(0, ElevatorKp);
    // motor.config_kI(0, ElevatorKi);
    // motor.config_kD(0, ElevatorKd);


    //config.supplyCurrLimit.currentLimit();
    motor.configAllSettings(config);
    motor.selectProfileSlot(0, 0);

    // motor.config_kF(0, ElevatorKf);
    // motor.config_kP(0, ElevatorKp);
    // motor.config_kI(0, ElevatorKi);
    // motor.config_kD(0, ElevatorKd);

    // motor.configMotionCruiseVelocity(ElevatorVelLimit);
    // motor.configMotionAcceleration(ElevatorAccelLimit);
    // motor.configMotionSCurveStrength(ElevatorSCurveStrength);
  }

  public void setPosition(double ticks){
     motor.set(ControlMode.MotionMagic, ticks);
     // motor.set(TalonFXControlMode.MotionMagic, ticks);
    // System.out.println(ticks);
  }

  public void moveUp() {
    motor.set(ControlMode.PercentOutput, -.35);
  }

  public void moveDown() {
    motor.set(ControlMode.PercentOutput, .35);
  }

  public void stop() {
    motor.set(ControlMode.PercentOutput, 0);
  }

  public WPI_TalonFX getMotor() {
    return motor;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("elevatorPos", motor.getSelectedSensorPosition(0));
  }
}
