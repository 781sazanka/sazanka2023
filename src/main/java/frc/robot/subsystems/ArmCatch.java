// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.ArmCatchConstants.*;
import frc.robot.Constants.Direction;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.ExternalFollower;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *  @review finished(3/18 9:50)
 * @test 3/17 going to test
 */
public class ArmCatch extends SubsystemBase {
  private final CANSparkMax[] Motor = {new CANSparkMax(LeftID, CANSparkMaxLowLevel.MotorType.kBrushless),
                                       new CANSparkMax(RightID, CANSparkMaxLowLevel.MotorType.kBrushless)};
  private RelativeEncoder[] Encoder = {Motor[Direction.Left.getCode()].getEncoder(),
                                       Motor[Direction.Right.getCode()].getEncoder()};
  private double[] CurrentPosition = {ArmFarPose,ArmFarPose};
  
  private double[] leftVel = {0,0};
  private double[] rightVel = {0,0};
  private State[] motorState = {State.Disabled,State.Disabled};
  private boolean[] isReached = {false,false};
  private int cnt = 0;

  private ArmCatch() {
    Motor[Direction.Left.getCode()].restoreFactoryDefaults();
    Motor[Direction.Right.getCode()].restoreFactoryDefaults();
    Motor[Direction.Left.getCode()].setIdleMode(CANSparkMax.IdleMode.kBrake);
    Motor[Direction.Right.getCode()].setIdleMode(CANSparkMax.IdleMode.kBrake);
    //TODO: figure out whether should invert this
    Motor[Direction.Left.getCode()].setInverted(false);
    Motor[Direction.Right.getCode()].setInverted(false);
    Motor[Direction.Left.getCode()].setSmartCurrentLimit(30);
    Motor[Direction.Right.getCode()].setSmartCurrentLimit(30);

    Encoder[Direction.Left.getCode()].setPositionConversionFactor(kEncoderDistancePerPulse);
    Encoder[Direction.Left.getCode()].setVelocityConversionFactor(kEncoderDistancePerPulse);
    Encoder[Direction.Right.getCode()].setPositionConversionFactor(kEncoderDistancePerPulse);
    Encoder[Direction.Right.getCode()].setVelocityConversionFactor(kEncoderDistancePerPulse);

    Encoder[Direction.Left.getCode()].setMeasurementPeriod(50);
    Encoder[Direction.Right.getCode()].setMeasurementPeriod(50);

    motorState[Direction.Left.getCode()] = motorState[Direction.Right.getCode()] = State.Disabled;
  }

  public ArmCatch(boolean isInitialize) {
    this();
    if (isInitialize) {
      resetEncoders();
      resetPositions();
    }
  }

  public void parameterIinit() {
    leftVel[0] = leftVel[1] = 0;
    rightVel[0] = rightVel[1] = 0;
    motorState[Direction.Left.getCode()] = motorState[Direction.Right.getCode()] = State.UnReached;
    cnt = 0;
    isReached[0] = isReached[1] = false;
  }

  //fin
  public void resetEncoders(){
    Encoder[Direction.Left.getCode()].setPosition(ArmFarPose);
    Encoder[Direction.Right.getCode()].setPosition(ArmFarPose);    
  }
  //fin
  public void resetPositions() {
    this.CurrentPosition[Direction.Left.getCode()] = this.CurrentPosition[Direction.Right.getCode()] = ArmFarPose;
  }

  // もしleft arm がターゲットに到達したら、加速度が減少するので、それを検知する
  //fin
  public boolean isLeftReached() {
    if(getAccel()[Direction.Left.getCode()] < AccelerationThreshold) 
      motorState[Direction.Left.getCode()] = State.Reached;
    
    if(motorState[Direction.Left.getCode()] == State.Reached){
      return true;
    } else {
      return false;
    }
  }

  // もしright arm がターゲットに到達したら、加速度が減少するので、それを検知する
  //fin
  public boolean isRightReached() {
    if(getAccel()[Direction.Right.getCode()] < AccelerationThreshold)
      motorState[Direction.Right.getCode()] = State.Reached;

    if(motorState[Direction.Right.getCode()] == State.Reached){
      return true;
    } else {
      return false;
    }
  }
  //fin
  public boolean isBothReached() {
    return (motorState[Direction.Left.getCode()] == State.Reached) && (motorState[Direction.Right.getCode()] == State.Reached);
  }

  //fin
  public void executeLeftReached() {
    Motor[Direction.Left.getCode()].follow(ExternalFollower.kFollowerDisabled, RightID);
    setMotorLeftVolt(0);
    if(motorState[Direction.Right.getCode()] != State.Reached)
      setMotorRightVolt(0.1);
  }
  //fin
  public void executeRightReached() {
    Motor[Direction.Left.getCode()].follow(ExternalFollower.kFollowerDisabled, RightID);
    setMotorRightVolt(0);
    if(motorState[Direction.Left.getCode()] != State.Reached)
      setMotorLeftVolt(0.1);
  }
  //fin
  public void executeBothReached() {
    //TODO: set the invert direction / the direction should be opposite
    Motor[Direction.Right.getCode()].follow(Motor[Direction.Left.getCode()],false);
    Commands.runOnce(() -> setMotorLeftVolt(0.1), this)
    .withTimeout(0.5)
    .andThen(() -> {Motor[Direction.Left.getCode()].stopMotor(); Motor[Direction.Right.getCode()].stopMotor();})
    .andThen(() -> reachTheSetPointWithFollow(ArmLeftMediumPose))
    .andThen(() -> setSetPoint( getMeasurement()[Direction.Left.getCode()]
                               ,getMeasurement()[Direction.Right.getCode()]));
  }
  //fin
  public void executeFollow() {
    //TODO: make sure the direction is opposite
    Motor[Direction.Right.getCode()].follow(Motor[Direction.Left.getCode()],false);
    setMotorLeftVolt(0.2);
  }

  public boolean executeReturnToMedium() {
    Motor[Direction.Right.getCode()].follow(Motor[Direction.Left.getCode()],false);
    if(Math.abs(getMeasurement()[Direction.Left.getCode()]-getLeftSetPoint()) > Math.abs(Tolerance)) {
      if((getMeasurement()[Direction.Left.getCode()]-getLeftSetPoint()) > 0) {
        Motor[Direction.Left.getCode()].set(-0.1);
      } else {
        Motor[Direction.Left.getCode()].set(0.1);
      }
      return false;
    } else {
      Motor[Direction.Left.getCode()].follow(ExternalFollower.kFollowerDisabled, RightID);
      Motor[Direction.Left.getCode()].set(0);
      Motor[Direction.Right.getCode()].set(0);
      return true;
    }
  }

  public boolean executeReturnToDefault() {
    
    if(Math.abs(getMeasurement()[Direction.Left.getCode()]-getLeftSetPoint()) > Math.abs(Tolerance)) {
      if((getMeasurement()[Direction.Left.getCode()]-getLeftSetPoint()) > 0) {
        Motor[Direction.Left.getCode()].set(-0.1);
      } else {
        Motor[Direction.Left.getCode()].set(0.1);
      }
    } else {
      Motor[Direction.Left.getCode()].set(0);
      isReached[Direction.Left.getCode()] = true;
    }
    if(Math.abs(getMeasurement()[Direction.Left.getCode()]-getRightSetPoint()) > Math.abs(Tolerance)) {
      if((getMeasurement()[Direction.Right.getCode()]-getRightSetPoint()) > 0) {
        Motor[Direction.Right.getCode()].set(-0.1);
      } else {
        Motor[Direction.Right.getCode()].set(0.1);
      }
    } else {
      Motor[Direction.Right.getCode()].set(0);
      isReached[Direction.Right.getCode()] = true;
    }
    return isReached[Direction.Right.getCode()] && isReached[Direction.Left.getCode()];
  }

  //fin
  public double[] getVelocity() {
    double[] vel = {Encoder[Direction.Left.getCode()].getVelocity(),Encoder[Direction.Right.getCode()].getVelocity()};
    return vel;
  }
  //fin
  public double[] getMeasurement() {
    double[] measurement = {Encoder[Direction.Left.getCode()].getPosition(),Encoder[Direction.Right.getCode()].getPosition()};
    return measurement;
  }
  //fin
  private double[] calcAccel(double leftAfter, double leftPrev, double rightAfter, double rightPrev) {
    double[] accel = {(leftAfter-leftPrev) / Encoder[Direction.Left.getCode()].getMeasurementPeriod()
                      ,(rightAfter-rightPrev) /Encoder[Direction.Right.getCode()].getMeasurementPeriod()};
    return accel;
  }
  //fin
  public double[] getAccel() {
    double[] acceleration;
    if(cnt++/2 == 0) {
      leftVel[0] = getVelocity()[Direction.Left.getCode()];
      rightVel[0] = getVelocity()[Direction.Right.getCode()];
      acceleration = calcAccel(leftVel[0], leftVel[1], rightVel[0], rightVel[1]);

    } else {
      leftVel[1] = getVelocity()[Direction.Left.getCode()];
      rightVel[1] = getVelocity()[Direction.Right.getCode()];
      acceleration = calcAccel(leftVel[1], leftVel[0], rightVel[1], rightVel[0]);
    }
    return acceleration;
  }
  //fin
  public double getLeftSetPoint() {
    return this.CurrentPosition[Direction.Left.getCode()];
  }
  //fin
  public double getRightSetPoint() {
    return this.CurrentPosition[Direction.Right.getCode()];
  }
  //fin
  public void setSetPoint(double leftSetPointInMeters, double rightSetPointInMeters){
    this.CurrentPosition[Direction.Left.getCode()] = leftSetPointInMeters;
    this.CurrentPosition[Direction.Right.getCode()] = rightSetPointInMeters;
  }
  //fin
  public void setMotorLeftVolt (double volt) {
    if((getMeasurement()[Direction.Left.getCode()] > ArmFarPose) && (getMeasurement()[Direction.Left.getCode()] < ArmNearPose))
      Motor[Direction.Left.getCode()].set(volt);
    else
      Motor[Direction.Left.getCode()].stopMotor();
  }
  //fin
  public void setMotorRightVolt (double volt) {
    if((getMeasurement()[Direction.Right.getCode()] > ArmFarPose) && (getMeasurement()[Direction.Right.getCode()] < ArmNearPose))
      Motor[Direction.Right.getCode()].set(volt);
    else
      Motor[Direction.Right.getCode()].stopMotor();
  }

  public void reachTheSetPointWithFollow(double leftSetPoint) {
    //TODO: make sure this will go the same direction
    Motor[Direction.Right.getCode()].follow(Motor[Direction.Left.getCode()],false);
    while(Math.abs(leftSetPoint - getMeasurement()[Direction.Left.getCode()]) > Math.abs(Tolerance)) {
      if(leftSetPoint > getMeasurement()[Direction.Left.getCode()])
        setMotorLeftVolt(0.1);
      else 
        setMotorLeftVolt(-0.1);
    }
    setMotorLeftVolt(0);
    setMotorRightVolt(0);
  }
  
  //fin
  public void getConvertedEncoderData() {
    SmartDashboard.putNumber("ArmRotation Left Position [m]", Encoder[Direction.Left.getCode()].getPosition());
    SmartDashboard.putNumber("ArmRotation Left Velocity [m/s]", Encoder[Direction.Left.getCode()].getVelocity());
    SmartDashboard.putNumber("ArmRotation Right Position [m]", Encoder[Direction.Right.getCode()].getPosition());
    SmartDashboard.putNumber("ArmRotation Right Velocity [m/s]", Encoder[Direction.Right.getCode()].getVelocity());
  }
}
