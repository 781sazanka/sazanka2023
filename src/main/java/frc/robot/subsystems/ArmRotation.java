// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//TODO: make sure to check the direction of encoder / CW:increase CCW:decrease
package frc.robot.subsystems;

import frc.robot.Constants.ArmRotationConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 *  @review finished(3/18 9:50)
 *  @test 3/17 going to test
 */
public class ArmRotation extends SubsystemBase{
  private final CANSparkMax motor = new CANSparkMax(ArmRotationConstants.ID,
    CANSparkMaxLowLevel.MotorType.kBrushless);
  private RelativeEncoder encoder = motor.getEncoder();

  private double setPointInRads = 0;

  private ArmRotation() {
    motor.restoreFactoryDefaults();
    motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    motor.setSmartCurrentLimit(40);
    //TODO: figure out whether should invert this
    motor.setInverted(true);
    //TODO:figure out the direction of encoder / should invert the conversion factor??
    encoder.setPositionConversionFactor(Units.rotationsToRadians(1) / ArmRotationConstants.ArmGearRatio);
    encoder.setVelocityConversionFactor(Units.rotationsToRadians(1) / ArmRotationConstants.ArmGearRatio);
  }

  public ArmRotation(boolean isInitialize) {
    this();
    if (isInitialize) {
      resetEncoders();
      resetPositions();
    }
  }

  public double getMeasurement() {
    return encoder.getPosition();
  }

  // only this method is for controling motor via PID control
  public void runWithSetPoint(double setPointInRads){
    this.setPointInRads = setPointInRads;
    if(setPointInRads > 0){
      if(getMeasurement() < setPointInRads/3)
        setMotorVolt(0.4);
      else if(getMeasurement() < setPointInRads/3*2)
        setMotorVolt(0.2);
      else if(getMeasurement() <= setPointInRads)
        setMotorVolt(0.1);
      else
        stop();
    } else {
      if(getMeasurement() > setPointInRads/3)
      setMotorVolt(-0.4);
      else if(getMeasurement() > setPointInRads/3*2)
        setMotorVolt(-0.2);
      else if(getMeasurement() >= setPointInRads)
        setMotorVolt(-0.1);
      else
        stop();
    }
  }

  // private void setSetPoint(double setPointInRads) {
  //   this.setPointInRads = setPointInRads;
  // }

  public double getSetPoint() {
    return this.setPointInRads;
  }

  public void resetEncoders(){
    encoder.setPosition(ArmRotationConstants.ArmButtomRads);
  }

  public void resetPositions() {
    this.setPointInRads = ArmRotationConstants.ArmButtomRads;
  }

  public boolean checkAtSetPoint() {
    if(Math.abs(getMeasurement()-getSetPoint()) < Math.abs(ArmRotationConstants.Tolerance)){
      return true;
    } else {
      return false;
    }
  }

  /**
   * @param volt Outside:increase / Inside:decrease
   */
  public void setMotorVolt(double volt){
    motor.set(volt);
  }
  public void stop(){
    motor.stopMotor();
  }
  public void getConvertedEncoderData() {
    SmartDashboard.putNumber("ArmRotation Encoder Position [rad]", encoder.getPosition());
    SmartDashboard.putNumber("ArmRotation Encoder Velocity [rad/s]", encoder.getVelocity());
  }
}


/*
 * The below program is the previous version
 */

// //TODO: make sure to check the direction of encoder / CW:increase CCW:decrease
// package frc.robot.subsystems;

// import frc.robot.Constants.ArmRotationConstants;

// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMaxLowLevel;
// import com.revrobotics.RelativeEncoder;

// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.math.controller.ArmFeedforward;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

// /**
//  *  @review finished(3/18 9:50)
//  *  @test 3/17 going to test
//  */
// public class ArmRotation extends ProfiledPIDSubsystem implements ProfiledInterface{
//   private final CANSparkMax motor = new CANSparkMax(ArmRotationConstants.ID,
//     CANSparkMaxLowLevel.MotorType.kBrushless);
//   private RelativeEncoder encoder = motor.getEncoder();

//   private double setPointInRads = 0;

//   private final ArmFeedforward m_feedforward = new ArmFeedforward(
//       ArmRotationConstants.kSVolts, ArmRotationConstants.kGVolts,
//       ArmRotationConstants.kVVoltSecondPerRad, ArmRotationConstants.kAVoltSecondSquaredPerRad);

//   private ArmRotation() {
//     super(
//         new ProfiledPIDController(
//             ArmRotationConstants.kP,
//             ArmRotationConstants.kI,
//             ArmRotationConstants.kD,
//             new TrapezoidProfile.Constraints(
//                 ArmRotationConstants.kMaxVelocityRadPerSecond,
//                 ArmRotationConstants.kMaxAccelerationRadPerSecSquared)),
//         0);
//     getController().setTolerance(ArmRotationConstants.Tolerance);
    
//     motor.restoreFactoryDefaults();
//     motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
//     motor.setSmartCurrentLimit(40);
//     //TODO: figure out whether should invert this
//     motor.setInverted(true);
//     //TODO:figure out the direction of encoder / should invert the conversion factor??
//     encoder.setPositionConversionFactor(Units.rotationsToRadians(1) / ArmRotationConstants.ArmGearRatio);
//     encoder.setVelocityConversionFactor(Units.rotationsToRadians(1) / ArmRotationConstants.ArmGearRatio);
//   }

//   public ArmRotation(boolean isInitialize) {
//     this();
//     if (isInitialize) {
//       resetEncoders();
//       resetPositions();
//     }
//   }

//   @Override
//   public void useOutput(double output, TrapezoidProfile.State setpoint) {
//     double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
//     motor.setVoltage(output + feedforward);
//   }

//   @Override
//   public double getMeasurement() {
//     return encoder.getPosition();
//   }

//   // only this method is for controling motor via PID control
//   public void runWithSetPoint(double setPointInRads){
//     setGoal(setPointInRads);
//     this.setPointInRads = setPointInRads;
//     enable();
//   }

//   public double getSetPoint() {
//     return this.setPointInRads;
//   }

//   public void resetEncoders(){
//     encoder.setPosition(ArmRotationConstants.ArmButtomRads);
//   }

//   public void resetPositions() {
//     this.setPointInRads = ArmRotationConstants.ArmButtomRads;
//   }

//   public boolean isGoal() {
//     return getController().atSetpoint();
//   }

//   /* 
//    * the methods below are for radio control and testing
//    * make sure to call disable method when use this method to control.
//    */
//   public void setMotorVolt(double volt){
//     disable();
//     motor.set(volt);
//   }
//   public void stop(){
//     disable();
//     motor.stopMotor();
//   }
//   public void getConvertedEncoderData() {
//     disable();
//     SmartDashboard.putNumber("ArmRotation Encoder Position [rad]", encoder.getPosition());
//     SmartDashboard.putNumber("ArmRotation Encoder Velocity [rad/s]", encoder.getVelocity());
//   }
// }
