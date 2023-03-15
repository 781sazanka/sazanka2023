// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Drivetrain;

public class DriveTrain extends SubsystemBase {
  private final WPI_TalonFX leftMotor1 = new WPI_TalonFX(Drivetrain.MOTOR_LEFT_1);
  private final WPI_TalonFX leftMotor2 = new WPI_TalonFX(Drivetrain.MOTOR_LEFT_2);
  private final WPI_TalonFX rightMotor1 = new WPI_TalonFX(Drivetrain.MOTOR_RIGHT_1);
  private final WPI_TalonFX rightMotor2 = new WPI_TalonFX(Drivetrain.MOTOR_RIGHT_2);

  private final DifferentialDrive m_drive = new DifferentialDrive(leftMotor1,rightMotor1);
  private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(new Rotation2d(),0,0,new Pose2d());
  private final AHRS navX = new AHRS();

  public DriveTrain() {
    
    leftMotor2.follow(leftMotor1);
    rightMotor2.follow(rightMotor1);

    rightMotor1.setInverted(false);
    rightMotor2.setInverted(false);
    leftMotor1.setInverted(true);
    leftMotor2.setInverted(true);
    // rightMotor1.enableVoltageCompensation(true);
    // rightMotor2.enableVoltageCompensation(true);
    // leftMotor1.enableVoltageCompensation(true);
    // leftMotor2.enableVoltageCompensation(true);

    setNeutralMode(NeutralMode.Brake);

    resetEncoders();
    resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
  }

  @Override
  public void periodic() {
    odometry.update(
        Rotation2d.fromDegrees(getRotation()),
        // getting the traveled distance by leftmotor and rightmotor
        leftMotor1.getSelectedSensorPosition()
            / Drivetrain.SENSOR_UNITS_PER_METER,
        rightMotor1.getSelectedSensorPosition()
            / Drivetrain.SENSOR_UNITS_PER_METER);
    // @question how to utilize field2D class?? / showing the current pose in
    // dashboards
    // RobotContainer.field.setRobotPose(getPose());
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        leftMotor1.getSelectedSensorVelocity()
            / Drivetrain.SENSOR_UNITS_PER_METER,
        rightMotor1.getSelectedSensorVelocity()
            / Drivetrain.SENSOR_UNITS_PER_METER);
  }

  public void resetEncoders() {
    leftMotor1.setSelectedSensorPosition(0);
    rightMotor1.setSelectedSensorPosition(0);
    navX.reset();
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(navX.getRotation2d(), 
      0,
      0,
      pose);
  }

  public void tankDriveVolts(double left, double right) {
    leftMotor1.setVoltage(left);
    rightMotor1.setVoltage(right);
    leftMotor1.feed();
    rightMotor1.feed();
  }

  public void stop() {
    // set the output voltage to zero
    // leftMotor1.set(ControlMode.PercentOutput, 0);
    // rightMotor1.set(ControlMode.PercentOutput, 0);
    this.tankDriveVolts(0, 0);
  }

  public void arcadeDrive(double fwd, double rot){
    m_drive.arcadeDrive(fwd, rot);
  }

  /**
   * @param left  the percent of output of the left motor (-1 to 1)
   * @param right the percent of output of the right motor (-1 to 1)
   */
  public void tankDrive(double left, double right) {
    m_drive.tankDrive(left, right);
  }

  public void curvatureDrive(double xSpeed, double zRotation, boolean allowTurnInPlace){
    m_drive.curvatureDrive(xSpeed, zRotation, allowTurnInPlace);
  }

   // public void arcadeDrive(double xSpeed, double zRotation) {
  //   //copySign(a,b) a:absolute value, b:sign(plus or minus)
  //   double turnDiff = Math.copySign(Math.pow(zRotation, 2), zRotation)
  //       / ((Math.pow(xSpeed, 2) / 3) + 0.5);
  //   double leftMotorOutput = Math.copySign(Math.pow(xSpeed, 2), -xSpeed) + turnDiff;
  //   double rightMotorOutput = Math.copySign(Math.pow(xSpeed, 2), -xSpeed) - turnDiff;

  //   leftMotor1.set(leftMotorOutput);
  //   rightMotor1.set(rightMotorOutput);
  // }

  public void setNeutralMode(NeutralMode mode) {
    leftMotor1.setNeutralMode(mode);
    leftMotor2.setNeutralMode(mode);
    rightMotor1.setNeutralMode(mode);
    rightMotor2.setNeutralMode(mode);
  }
  // why reverse the rotation ( now CW will increase, CCW will decrease)
  private double getRotation() {
    return -navX.getRotation2d().getDegrees();
  }
  
  //TODO: find a way to set a voltage constraints during trajectory following
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }
}
