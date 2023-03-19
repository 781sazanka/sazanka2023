// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * TODO: make sure the encoder value is CW;plus, CCW:minus
 * ATTENTION: the encoder value is "0" for the shortest position
 */
package frc.robot.subsystems;

import frc.robot.Constants.SliderConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * @review finished
 * @test 3/17 going to test
 */
public class Slider extends ProfiledPIDSubsystem implements ProfiledInterface{
  private final CANSparkMax left_motor = new CANSparkMax(6,
    CANSparkMaxLowLevel.MotorType.kBrushless);
  // private final CANSparkMax right_motor = new CANSparkMax(SliderConstants.RightMotorID,
  //   CANSparkMaxLowLevel.MotorType.kBrushless);
  private RelativeEncoder encoder = left_motor.getEncoder();

  private double setPointInMeters = SliderConstants.SliderShortestInMeters;

  //TODO: what feedforward class is suite for Slider
  private final ArmFeedforward m_feedforward = new ArmFeedforward(
      SliderConstants.kSVolts, SliderConstants.kGVolts,
      SliderConstants.kVVoltSecondPerRad, SliderConstants.kAVoltSecondSquaredPerRad);

  private Slider() {
    super(
        new ProfiledPIDController(
            SliderConstants.kP,
            SliderConstants.kI,
            SliderConstants.kD,
            new TrapezoidProfile.Constraints(
                SliderConstants.kMaxVelocityMeterPerSecond,
                SliderConstants.kMaxAccelerationMeterPerSecSquared)),
        0);
    getController().setTolerance(SliderConstants.Tolerance);
    
    left_motor.restoreFactoryDefaults();
    // right_motor.restoreFactoryDefaults();
    left_motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    // right_motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    // left_motor.setSmartCurrentLimit(40);
    // right_motor.setSmartCurrentLimit(40);

    // right_motor.follow(left_motor, false);
    //TODO: figure out whether should invert this
    left_motor.setInverted(false);
    
    encoder.setPositionConversionFactor(SliderConstants.kEncoderDistancePerPulse);
    encoder.setVelocityConversionFactor(SliderConstants.kEncoderDistancePerPulse);
  }

  public Slider(boolean isInitialize){
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
    //TODO: figure out the direction / make sure the data will increase as slider becomes longer
    return encoder.getPosition();
  }

  /**
   * @param radians longer:increase / shorter:decrease
   */
  public void runWithSetPoint(double setPointInMeters){
    setGoal(setPointInMeters);
    this.setPointInMeters = setPointInMeters;
    enable();
  }

  public double getSetPoint() {
    return this.setPointInMeters;
  }

  public void resetEncoders(){
    encoder.setPosition(SliderConstants.SliderShortestInMeters);
  }

  public void resetPositions() {
    this.setPointInMeters = SliderConstants.SliderShortestInMeters;
  }

  public boolean isGoal() {
    return getController().atGoal();
  }

  /* 
   * the methods below are for radio control and testing
   * make sure to call disable method when use this method to control.
   */
  public void setMotorVolt(double volt, boolean inverted){
    disable();
    left_motor.setInverted(inverted);
    left_motor.set(volt);
  }

  public void stop(){
    disable();
    left_motor.stopMotor();
    // right_motor.stopMotor();
  }
  public void getConvertedEncoderData() {
    disable();
    SmartDashboard.putNumber("Slider Encoder Position [m]", encoder.getPosition());
    SmartDashboard.putNumber("Slider Encoder Velocity [m/s]", encoder.getVelocity());
  }
}
