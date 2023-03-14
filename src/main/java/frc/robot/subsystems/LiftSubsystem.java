// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.LiftConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

/** A robot arm subsystem that moves with a motion profile. */
public class LiftSubsystem extends ProfiledPIDSubsystem {
  private final CANSparkMax left_motor = new CANSparkMax(LiftConstants.LeftMotorID,
    CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax right_motor = new CANSparkMax(LiftConstants.RightMotorID,
    CANSparkMaxLowLevel.MotorType.kBrushless);
  private RelativeEncoder encoder = left_motor.getEncoder();

  private final ArmFeedforward m_feedforward = new ArmFeedforward(
      LiftConstants.kSVolts, LiftConstants.kGVolts,
      LiftConstants.kVVoltSecondPerRad, LiftConstants.kAVoltSecondSquaredPerRad);

  /** Create a new ArmSubsystem. */
  public LiftSubsystem() {
    super(
        new ProfiledPIDController(
            LiftConstants.kP,
            LiftConstants.kI,
            LiftConstants.kD,
            new TrapezoidProfile.Constraints(
                LiftConstants.kMaxVelocityRadPerSecond,
                LiftConstants.kMaxAccelerationRadPerSecSquared)),
        0);
    
    left_motor.restoreFactoryDefaults();
    right_motor.restoreFactoryDefaults();

    left_motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    right_motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    left_motor.setSmartCurrentLimit(40);
    right_motor.setSmartCurrentLimit(40);
    right_motor.follow(left_motor, true);
    
    //TODO: changing these settings
    left_motor.getEncoder().setPositionConversionFactor(Units.rotationsToRadians(1) / 151.2);  
    encoder.setPositionConversionFactor(Units.rotationsToRadians(1) * LiftConstants.LiftGearRatio);
    encoder.setVelocityConversionFactor(Units.rotationsToRadians(1) * LiftConstants.LiftGearRatio);

    left_motor.setInverted(false);
    encoder.setInverted(false);

    // Start arm at rest in neutral position / 初期のポジションを真ん中に設定(not 0)
    setGoal(LiftConstants.ArmOffsetRads);
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Calculate the feedforward from the sepoint
    double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
    // Add the feedforward to the PID output to get the motor output
    left_motor.setVoltage(output + feedforward);
  }

  @Override
  public double getMeasurement() {
    //TODO: make it return a radian value
    return encoder.getPosition() + LiftConstants.ArmOffsetRads;
  }

  public void setLeftMotorVolt(double volt, boolean inverted){
    left_motor.setInverted(inverted);
    left_motor.set(volt);
  }

  public void stop(){
    left_motor.stopMotor();
    right_motor.stopMotor();
  }

}
