// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * the encoder value is extending:increase, shrinking:decrease
 * ATTENTION: the encoder value is "0" for extended position,
 */

 /**
  * @review finished (3/22 23:00)
  */
package frc.robot.subsystems;

import static frc.robot.Constants.LiftConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * @review finished
 * @test 3/17 going to test
 * Don't need to initialize any parameters each execution
 */
public class Lift extends SubsystemBase{
  private final CANSparkMax leftMotor = new CANSparkMax(LeftMotorID,
    CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax rightMotor = new CANSparkMax(RightMotorID,
    CANSparkMaxLowLevel.MotorType.kBrushless);
  private RelativeEncoder encoder = leftMotor.getEncoder();
  
  private double setPointInRads = LiftHorizontalPos;

  private Lift() {
    leftMotor.restoreFactoryDefaults();
    rightMotor.restoreFactoryDefaults();
    leftMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    rightMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    rightMotor.follow(leftMotor, true);
    leftMotor.setInverted(true);
    
    encoder.setPositionConversionFactor(Units.rotationsToRadians(1) / LiftGearRatio);
    encoder.setVelocityConversionFactor(Units.rotationsToRadians(1) / LiftGearRatio);
  }

  public Lift(boolean isInitialize){
    this();
    if(isInitialize) {
      resetEncoders();
      resetPositions();
    }
  }

  public void executeUp() {
    if (getMeasurement() < LiftExtendedPos/3) {
      setMotorVolt(0.3);
    } else if(LiftExtendedPos/3 <= getMeasurement() && getMeasurement() <= LiftExtendedPos/3*2){
      setMotorVolt(0.15);
    } else {
      setMotorVolt(0.1);
    }
  }

  public void executeDown() {
    if (getMeasurement() > LiftExtendedPos/3*2) {
      setMotorVolt(-0.05);
    } else if(LiftExtendedPos/3 <= getMeasurement() && getMeasurement() <= LiftExtendedPos/3*2){
      setMotorVolt(-0.03);
    } else {
      setMotorVolt(0.05);
    }
  }

  public void executeStay() {
    if(!isSetPoint())
      setMotorVolt(kP*(getSetPoint()-getMeasurement()));  // simple feedforward [kp * rad], KP should be positive
  }

  public double getMeasurement() {
    return encoder.getPosition();
  }

  public double getSetPoint() {
    return this.setPointInRads;
  }

  public void setSetPoint(double setPointInRads) {
    this.setPointInRads = setPointInRads;
  }

  public void resetEncoders(){
    encoder.setPosition(LiftHorizontalPos);
  }

  public void resetPositions() {
    this.setPointInRads = LiftHorizontalPos;
  }

  // return true if the error is within the tolerance
  public boolean isSetPoint() {
    return (Math.abs(getMeasurement() - getSetPoint()) < Math.abs(Tolerance));
  }

  /* 
   * the methods below are for radio control and testing
   * make sure to call disable method when use this method to control.
   */
  public void setMotorVolt(double volt){
    if ((LiftHorizontalPos <= getMeasurement()) && (getMeasurement() <= LiftExtendedPos)) {
      leftMotor.set(volt);
      // SmartDashboard.putNumber("voltage setpoint [V]", volt);
      // SmartDashboard.putNumber("applied voltage [V]", leftMotor.getAppliedOutput());
    } else {
      // SmartDashboard.putString("Lift State", "WARNING :: Exceeded the limit!!!");
      //TODO: it might be better to reverse the motor for a second
      stop();
    }
  }

  public void stop(){
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }
  public void getConvertedEncoderData() {
    SmartDashboard.putNumber("ArmRotation Encoder Position [rad]", encoder.getPosition());
    SmartDashboard.putNumber("ArmRotation Encoder Velocity [rad/s]", encoder.getVelocity());
  }
}
