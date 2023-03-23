// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * TODO: make sure the encoder value is CW;plus, CCW:minus
 * ATTENTION: the encoder value is "0" for the shortest position
 */
package frc.robot.subsystems;

import static frc.robot.Constants.SliderConstants.*;
import frc.robot.Constants.Direction;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.ExternalFollower;

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
  private final CANSparkMax[] Motor = { new CANSparkMax(LeftMotorID, CANSparkMaxLowLevel.MotorType.kBrushless),
                                        new CANSparkMax(RightMotorID, CANSparkMaxLowLevel.MotorType.kBrushless)};
private RelativeEncoder[] Encoder = { Motor[Direction.Left.getCode()].getEncoder(),
                                      Motor[Direction.Right.getCode()].getEncoder()};


  private double setPointInMeters = SliderShortestInMeters;

  //TODO: what feedforward class is suite for Slider
  private final ArmFeedforward m_feedforward = new ArmFeedforward(
      kSVolts, kGVolts,
      kVVoltSecondPerRad, kAVoltSecondSquaredPerRad);

  private Slider() {
    super(
        new ProfiledPIDController(
          kP,
          kI,
          kD,
          new TrapezoidProfile.Constraints(
              kMaxVelocityMeterPerSecond,
              kMaxAccelerationMeterPerSecSquared)),
        0);
    getController().setTolerance(Tolerance);
    
    Motor[Direction.Left.getCode()].restoreFactoryDefaults();
    Motor[Direction.Right.getCode()].restoreFactoryDefaults();
    Motor[Direction.Left.getCode()].setIdleMode(CANSparkMax.IdleMode.kBrake);
    Motor[Direction.Right.getCode()].setIdleMode(CANSparkMax.IdleMode.kBrake);
    Motor[Direction.Left.getCode()].setInverted(true);
    Motor[Direction.Right.getCode()].setInverted(true);
    Motor[Direction.Left.getCode()].setSmartCurrentLimit(30);
    Motor[Direction.Right.getCode()].setSmartCurrentLimit(30);

    Encoder[Direction.Left.getCode()].setPositionConversionFactor(kEncoderDistancePerPulse);
    Encoder[Direction.Left.getCode()].setVelocityConversionFactor(kEncoderDistancePerPulse);
    Encoder[Direction.Right.getCode()].setPositionConversionFactor(kEncoderDistancePerPulse);
    Encoder[Direction.Right.getCode()].setVelocityConversionFactor(kEncoderDistancePerPulse);

    //TODO: make sure as volt of leftmotor increases slider will extend
    Motor[Direction.Right.getCode()].follow(Motor[Direction.Left.getCode()], true);
    Motor[Direction.Left.getCode()].setInverted(true);
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
    boolean[] isExceed = isEncoderExceedLimit();

    if(isExceed[Direction.Left.getCode()]) {
      Motor[Direction.Left.getCode()].stopMotor();
    } else {
      Motor[Direction.Left.getCode()].setVoltage(output + feedforward);
    }
    if(isExceed[Direction.Right.getCode()]) {
      Motor[Direction.Left.getCode()].follow(ExternalFollower.kFollowerDisabled, RightMotorID);
      Motor[Direction.Right.getCode()].stopMotor();
    }
  } 

  @Override
  public double getMeasurement() {
    return Encoder[Direction.Left.getCode()].getPosition();
  }

  /**
   * @param radians longer:increase / shorter:decrease
   */
  public void runWithSetPoint(double setPointInMeters){
    setGoal(setPointInMeters);
    setSetPoint(setPointInMeters);
    enable();
  }
  public void setSetPoint(double setPointInMeters) {
    this.setPointInMeters = setPointInMeters;
  }
  
  public double getSetPoint() {
    return this.setPointInMeters;
  }

  public void resetEncoders(){
    Encoder[Direction.Left.getCode()].setPosition(SliderShortestInMeters);
  }

  public void resetPositions() {
    this.setPointInMeters = SliderShortestInMeters;
  }

  public boolean isSetPoint() {
    return getController().atSetpoint();
  }

  /* 
   * the methods below are for radio control and testing
   * make sure to call disable method when use this method to control.
   */
  public void setMotorVolt(double leftVolt, double rightVolt){
    disable();
    if(isEncoderExceedLimit()[Direction.Left.getCode()]){
      Motor[Direction.Left.getCode()].stopMotor();
    } else {
      Motor[Direction.Left.getCode()].set(leftVolt);
    }
    if(isEncoderExceedLimit()[Direction.Right.getCode()]){
      Motor[Direction.Right.getCode()].stopMotor();
    } else {
      Motor[Direction.Right.getCode()].set(rightVolt);
    }

  }
  //return true when encoder data exceed the limit
  public boolean[] isEncoderExceedLimit() {
    boolean[] isEncoderExceed = {false,false};
    isEncoderExceed[Direction.Left.getCode()] = !(SliderShortestInMeters <= Encoder[Direction.Left.getCode()].getPosition() && Encoder[Direction.Left.getCode()].getPosition() <= SliderLongestInMeters);
    isEncoderExceed[Direction.Right.getCode()] = !(SliderShortestInMeters <= Encoder[Direction.Right.getCode()].getPosition() && Encoder[Direction.Right.getCode()].getPosition() <= SliderLongestInMeters);
    return isEncoderExceed;
  }

  public void stop(){
    disable();
    Motor[Direction.Left.getCode()].stopMotor();
    Motor[Direction.Right.getCode()].stopMotor();
  }
  public void getConvertedEncoderData() {
    //TODO: make sure the encoder data will increase as it extends
    SmartDashboard.putNumber("Slider Encoder Position [m]", Encoder[Direction.Left.getCode()].getPosition());
    SmartDashboard.putNumber("Slider Encoder Velocity [m/s]", Encoder[Direction.Left.getCode()].getVelocity());
  }
}
