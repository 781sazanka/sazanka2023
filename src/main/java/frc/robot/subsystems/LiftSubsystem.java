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

  private double setPointInRads = 0;

  private final ArmFeedforward m_feedforward = new ArmFeedforward(
      LiftConstants.kSVolts, LiftConstants.kGVolts,
      LiftConstants.kVVoltSecondPerRad, LiftConstants.kAVoltSecondSquaredPerRad);

  private LiftSubsystem() {
    super(
        new ProfiledPIDController(
            LiftConstants.kP,
            LiftConstants.kI,
            LiftConstants.kD,
            new TrapezoidProfile.Constraints(
                LiftConstants.kMaxVelocityRadPerSecond,
                LiftConstants.kMaxAccelerationRadPerSecSquared)),
        0);
    getController().setTolerance(LiftConstants.Tolerance);
    
    left_motor.restoreFactoryDefaults();
    right_motor.restoreFactoryDefaults();
    left_motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    right_motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    left_motor.setSmartCurrentLimit(40);
    right_motor.setSmartCurrentLimit(40);

    right_motor.follow(left_motor, false);
    left_motor.setInverted(false);
    encoder.setInverted(false);
    
    //TODO: changing these settings
    encoder.setPositionConversionFactor(Units.rotationsToRadians(1) * LiftConstants.LiftGearRatio);
    encoder.setVelocityConversionFactor(Units.rotationsToRadians(1) * LiftConstants.LiftGearRatio);
    resetEncoders();
    // Start arm at rest in neutral position / 初期のポジションを真ん中に設定(not 0)
    // setGoal(LiftConstants.ArmOffsetRads);
  }

  public LiftSubsystem(boolean isInitialize){
    this();
    if(isInitialize) {
      resetPositions();
    }
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
    //TODO: make it return a radian value / figure out the direction
    return encoder.getPosition() + LiftConstants.LiftOffsetRads;
  }

  public void setMotorVolt(double volt, boolean inverted){
    left_motor.setInverted(inverted);
    left_motor.set(volt);
  }

  public void stop(){
    left_motor.stopMotor();
    right_motor.stopMotor();
  }

  /**
   * @param radians CW:plus CCW:minus
   */
  public void runWithSetPoint(double setPointInRads){
    setGoal(setPointInRads);
    this.setPointInRads = setPointInRads;
  }

  public double getSetPointInRads() {
    return this.setPointInRads;
  }

  private void resetEncoders(){
    encoder.setPosition(0);
  }

  private void resetPositions() {
    this.setPointInRads = 0;
  }
}
