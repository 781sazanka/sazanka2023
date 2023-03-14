// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.ArmRotationConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

/** A robot arm subsystem that moves with a motion profile. */
public class ArmRotationSubsystem extends ProfiledPIDSubsystem {
  private final CANSparkMax motor = new CANSparkMax(ArmRotationConstants.ID,
    CANSparkMaxLowLevel.MotorType.kBrushless);
  private RelativeEncoder encoder = motor.getEncoder();

  private final ArmFeedforward m_feedforward = new ArmFeedforward(
      ArmRotationConstants.kSVolts, ArmRotationConstants.kGVolts,
      ArmRotationConstants.kVVoltSecondPerRad, ArmRotationConstants.kAVoltSecondSquaredPerRad);

  /** Create a new ArmSubsystem. */
  public ArmRotationSubsystem() {
    super(
        new ProfiledPIDController(
            ArmRotationConstants.kP,
            ArmRotationConstants.kI,
            ArmRotationConstants.kD,
            new TrapezoidProfile.Constraints(
              ArmRotationConstants.kMaxVelocityRadPerSecond,
              ArmRotationConstants.kMaxAccelerationRadPerSecSquared)),
        0);
    
    motor.restoreFactoryDefaults();

    motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    motor.setSmartCurrentLimit(40);
    
    //TODO: changing these settings
    motor.getEncoder().setPositionConversionFactor(Units.rotationsToRadians(1) / 151.2);  
    encoder.setPositionConversionFactor(Units.rotationsToRadians(1) * ArmRotationConstants.ArmGearRatio);
    encoder.setVelocityConversionFactor(Units.rotationsToRadians(1) * ArmRotationConstants.ArmGearRatio);

    motor.setInverted(true);
    encoder.setInverted(true);

    // Start arm at rest in neutral position / 初期のポジションを真ん中に設定(not 0)
    setGoal(ArmRotationConstants.ArmOffsetRads);
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Calculate the feedforward from the sepoint
    double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
    // Add the feedforward to the PID output to get the motor output
    motor.setVoltage(output + feedforward);
  }

  @Override
  public double getMeasurement() {
    //TODO: make it return a radian value
    return encoder.getPosition() + ArmRotationConstants.ArmOffsetRads;
  }

  public void setMotorVolt(double volt, boolean inverted){
    motor.setInverted(inverted);
    motor.set(volt);
  }

  public void stop(){
    motor.stopMotor();
  }
}
