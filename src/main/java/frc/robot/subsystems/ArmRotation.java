// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// CW:increase CCW:decrease
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ArmRotationConstants.*;

/**
 *  @review finished(3/22 23:45)
 */
public class ArmRotation extends SubsystemBase{
  private final CANSparkMax motor = new CANSparkMax(ID,
    CANSparkMaxLowLevel.MotorType.kBrushless);
  private RelativeEncoder encoder = motor.getEncoder();

  private double setPointInRads = 0;

  private ArmRotation() {
    motor.restoreFactoryDefaults();
    motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    motor.setSmartCurrentLimit(40);
    motor.setInverted(true);
    encoder.setPositionConversionFactor(Units.rotationsToRadians(1) / ArmGearRatio);
    encoder.setVelocityConversionFactor(Units.rotationsToRadians(1) / ArmGearRatio);
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
    if(getSetPoint() > 0){
      if(getMeasurement() < getSetPoint()/4)
        setMotorVolt(0.25);
      else if((getSetPoint()/4 <= getMeasurement())  && (getMeasurement() <= getSetPoint()/4*3))
        setMotorVolt(0.5);
      else if((getMeasurement() > getSetPoint()/4*3) && !checkAtSetPoint())
        setMotorVolt(0.25);
      else
        stop();
    } else {
      if(getMeasurement() > getSetPoint()/4)
      setMotorVolt(-0.25);
      else if((getSetPoint()/4 >= getMeasurement())  && (getMeasurement() >= getSetPoint()/4*3))
        setMotorVolt(-0.5);
      else if((getMeasurement() <= getSetPoint()/4*3) && !checkAtSetPoint())
        setMotorVolt(-0.25);
      else
        stop();
    }
  }

  public void setSetPoint(double setPointInRads) {
    this.setPointInRads = setPointInRads;
  }

  public double getSetPoint() {
    return this.setPointInRads;
  }

  public void resetEncoders(){
    encoder.setPosition(ArmButtomRads);
  }

  public void resetPositions() {
    this.setPointInRads = ArmButtomRads;
  }

  public boolean checkAtSetPoint() {
    if(Math.abs(getMeasurement()-getSetPoint()) < Math.abs(Tolerance)){
      return true;
    } else {
      return false;
    }
  }

  /**
   * @param volt Outside:increase / Inside:decrease
   */
  public void setMotorVolt(double volt){
    if(ArmInsideMaxRads < getMeasurement()  && getMeasurement() < ArmOutsideMaxRads)
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
