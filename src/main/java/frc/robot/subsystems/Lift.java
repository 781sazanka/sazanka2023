// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * TODO: make sure the encoder value is CW;plus, CCW:minus
 * ATTENTION: the encoder value is "0" for extended position,
 */
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * @review finished
 * @test 3/17 going to test
 */
public class Lift extends ProfiledPIDSubsystem implements ProfiledInterface{
  private final CANSparkMax left_motor = new CANSparkMax(LiftConstants.LeftMotorID,
    CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax right_motor = new CANSparkMax(LiftConstants.RightMotorID,
    CANSparkMaxLowLevel.MotorType.kBrushless);
  private RelativeEncoder encoder = left_motor.getEncoder();
  
  private double setPointInRads = LiftConstants.LiftHorizontalPos;
  private boolean isReachedGoal = false;

  private final ArmFeedforward m_feedforward = new ArmFeedforward(
      LiftConstants.kSVolts, LiftConstants.kGVolts,
      LiftConstants.kVVoltSecondPerRad, LiftConstants.kAVoltSecondSquaredPerRad);

  private Lift() {
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

    right_motor.follow(left_motor, true);
    left_motor.setInverted(true);
    
    encoder.setPositionConversionFactor(Units.rotationsToRadians(1) / LiftConstants.LiftGearRatio);
    encoder.setVelocityConversionFactor(Units.rotationsToRadians(1) / LiftConstants.LiftGearRatio);
  }

  public Lift(boolean isInitialize){
    this();
    if(isInitialize) {
      resetEncoders();
      resetPositions();
    }
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
    left_motor.setVoltage(output + feedforward);
  }

  @Override
  public double getMeasurement() {
    return encoder.getPosition();
  }

  /**
   * @param radians CW:plus CCW:minus
   */
  public void runWithSetPoint(double setPointInRads){
    setGoal(setPointInRads);
    setSetPoint(setPointInRads);
    enable();
  }

  public double getSetPoint() {
    return this.setPointInRads;
  }

  public void setSetPoint(double setPointInRads) {
    this.setPointInRads = setPointInRads;
  }

  public void resetEncoders(){
    encoder.setPosition(LiftConstants.LiftHorizontalPos);
  }

  public void resetPositions() {
    this.setPointInRads = LiftConstants.LiftHorizontalPos;
  }

  public boolean isSetPoint() {
    return getController().atSetpoint();
    // return getController().atGoal() || isReachedGoal;
  }

  /* 
   * the methods below are for radio control and testing
   * make sure to call disable method when use this method to control.
   */
  public void setMotorVolt(double volt){
    disable();
    if ((encoder.getPosition() <= LiftConstants.LiftExtendedPos) && (encoder.getPosition() >= LiftConstants.LiftHorizontalPos)) {
      left_motor.set(volt);
    } else {
      // this.isReachedGoal = true;
      stop();
    }
  }

  public void stop(){
    disable();
    left_motor.stopMotor();
    right_motor.stopMotor();
  }
  public void getConvertedEncoderData() {
    disable();
    SmartDashboard.putNumber("ArmRotation Encoder Position [rad]", encoder.getPosition());
    SmartDashboard.putNumber("ArmRotation Encoder Velocity [rad/s]", encoder.getVelocity());
  }
}
