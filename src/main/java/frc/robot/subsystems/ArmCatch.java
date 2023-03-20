// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import frc.robot.Constants.ArmCatchConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *  @review finished(3/18 9:50)
 * @test 3/17 going to test
 */
public class ArmCatch extends ProfiledPIDSubsystem implements ProfiledInterface{
  private final CANSparkMax motor;
  private RelativeEncoder encoder;
  private int CANID;
  private boolean inverted;
  private double setPointInMeters = ArmCatchConstants.ArmFarPose;
  // DigitalInput farLimitSwitch;
  // DigitalInput nearLimitSwitch;

  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(
    ArmCatchConstants.kSVolts, ArmCatchConstants.kVVoltSecondPerRad, ArmCatchConstants.kAVoltSecondSquaredPerRad);
    
  private ArmCatch(boolean isLeft) {
    super(
        new ProfiledPIDController(
          ArmCatchConstants.kP,
          ArmCatchConstants.kI,
          ArmCatchConstants.kD,
          new TrapezoidProfile.Constraints(
              ArmCatchConstants.kMaxVelocityMeterPerSecond,
              ArmCatchConstants.kMaxAccelerationMeterPerSecSquared)),
        0);
    getController().setTolerance(ArmCatchConstants.Tolerance);

    if (isLeft) {
      this.motor = new CANSparkMax(ArmCatchConstants.LeftID, CANSparkMaxLowLevel.MotorType.kBrushless);
      this.CANID = ArmCatchConstants.LeftID;
    } else {
      this.motor = new CANSparkMax(ArmCatchConstants.RightID, CANSparkMaxLowLevel.MotorType.kBrushless);
      this.CANID = ArmCatchConstants.RightID;
    }

    this.encoder = this.motor.getEncoder();
    // //TODO: configuring the limit switches, true:when reaching the setpoint, false:when not reaching
    // if (isLeft) {
    //   farLimitSwitch = new DigitalInput(0);
    //   nearLimitSwitch = new DigitalInput(1);
    // } else {
    //   farLimitSwitch = new DigitalInput(2);
    //   nearLimitSwitch = new DigitalInput(3);
    // }

    motor.restoreFactoryDefaults();
    motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    motor.setSmartCurrentLimit(40);

    // TODO: make sure to invert motor direction so that positive is inside
    // TODO: make sure to encoder configure conversion factor so that positive is inside
    if (isLeft) {
      motor.setInverted(false);
      encoder.setPositionConversionFactor(ArmCatchConstants.kEncoderDistancePerPulse);
      encoder.setVelocityConversionFactor(ArmCatchConstants.kEncoderDistancePerPulse);  
    } else {
      motor.setInverted(false);
      encoder.setPositionConversionFactor(ArmCatchConstants.kEncoderDistancePerPulse);
      encoder.setVelocityConversionFactor(ArmCatchConstants.kEncoderDistancePerPulse);  
    }
  }

  public ArmCatch(Boolean isLeft,boolean isInitialize) {
    this(isLeft);
    if (isInitialize) {
      resetEncoders();
      resetPositions();
    }
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
    motor.setVoltage(output + feedforward);
  }

  @Override
  public double getMeasurement() {
    /*
     * TODO: changing the encoder value depending on the direction
     * this might not needed since it can be changed in conversion factor above
     */

    // if (this.CANID == ArmCatchConstants.LeftID) {
    //   return encoder.getPosition();
    // } else {
    //   return -encoder.getPosition();
    // }

    return encoder.getPosition();
  }

  /**
   * @param radians CW:plus CCW:minus
   */
  public void runWithSetPoint(double setPointInMeters){
    setGoal(setPointInMeters);
    setSetPoint(setPointInMeters);
    enable();
  }

  public double getSetPoint() {
    return this.setPointInMeters;
  }

  public String getMotorName() {
    if (this.CANID == ArmCatchConstants.LeftID) {
      return "Left";
    } else {
      return "Right";
    }
  }

  public void resetEncoders(){
    encoder.setPosition(ArmCatchConstants.ArmFarPose);
  }

  public void resetPositions() {
    this.setPointInMeters = ArmCatchConstants.ArmFarPose;
  }

  // public boolean getFarLimitSwitch() {
  //   return farLimitSwitch.get();
  // }

  // public boolean getNearLimitSwitch() {
  //   return nearLimitSwitch.get();
  // }

  public void setSetPoint(double setPointInMeters){
    this.setPointInMeters = setPointInMeters;
  }

  public boolean isGoal() {
    return getController().atSetpoint();
  }
  // //TODO: check whether the command is correct
  // public void reachFarLimitSwitch() {
  //   SmartDashboard.putBoolean(getMotorName() + "FarLimitSwitch", true);
  //   setSetPoint(ArmCatchConstants.ArmFarPose);
  //   Commands.runOnce(() -> setMotorVolt(-0.1,this.inverted), this)
  //     .withTimeout(0.3)
  //     .andThen(() -> stop(),this);
  // }

  // public void reachNearLimitSwitch() {
  //   SmartDashboard.putBoolean(getMotorName() + "NearLimitSwitch", true);
  //   setSetPoint(ArmCatchConstants.ArmNearPose);
  //   Commands.runOnce(() -> setMotorVolt(0.1,this.inverted), this)
  //   .withTimeout(0.3)
  //   .andThen(() -> stop(),this);
  // }
  
  /* 
   * the methods below are for radio control and testing
   * make sure to call disable method when use this method to control.
   */
  public void setMotorVolt(double motorVolt, boolean motorInverted){
    disable();
    motor.setInverted(motorInverted);
    //TODO: make sure the volt will increase when going inside
    motor.set(motorVolt);
  }

  public void stop(){
    disable();
    motor.stopMotor();
  }
  public void getConvertedEncoderData() {
    disable();
    String motorName = getMotorName();
    SmartDashboard.putNumber("ArmRotation" + motorName + "Position [m]", encoder.getPosition());
    SmartDashboard.putNumber("ArmRotation" + motorName + "Velocity [m/s]", encoder.getVelocity());
  }
}
