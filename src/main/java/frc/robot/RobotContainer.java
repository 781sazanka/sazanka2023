// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.*;
import frc.robot.Constants.LiftConstants.LiftOperateMode;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

public class RobotContainer {
  // Shuffleboard
  public static final ShuffleboardTab driveTab = Shuffleboard.getTab("DriveTrain");
  public static final ShuffleboardTab defaultCommandTab = Shuffleboard.getTab("defaulCommand State");
  public static Field2d field = new Field2d();                // in case to view the current place of robot

  public static double driveSpeedSensitivity = OperatorConstants.driverSpeedSensitivityDefault;
  public static double driveTurnSensitivity = OperatorConstants.driverTurnSensitivity;

  // controller
  public static final CommandXboxController m_driverController =
    new CommandXboxController(OperatorConstants.DriverControllerPort);
  public static final CommandXboxController m_mechanicsController =
    new CommandXboxController(OperatorConstants.MechanicsControllerPort);

  // subsystems
  private static final DriveTrain driveTrain = new DriveTrain();
  private static final Lift lift = new Lift(true);
  private static final ArmRotation armRotation = new ArmRotation(true);
  private static final ArmCatch armCatch = new ArmCatch(true);

  // commands
  private static final ArcadeDriveCommand ARCADE_DRIVE = 
    new ArcadeDriveCommand(driveTrain);

  private static String pathname;

  public RobotContainer() {
    configureBindings();

    // default commands settings
    driveTrain.setDefaultCommand(ARCADE_DRIVE);
    lift.setDefaultCommand(new LiftCommand(lift, LiftConstants.LiftOperateMode.Stay));
  }

  private void configureBindings() {
    m_driverController
    .rightBumper().debounce(0.5)
    .onTrue(new InstantCommand(() -> {driveSpeedSensitivity++; driveTurnSensitivity+=0.5;}));
    m_driverController
    .leftBumper().debounce(0.5)
    .onTrue(new InstantCommand(() -> {driveTurnSensitivity--; driveTurnSensitivity-=0.5;}));

    m_mechanicsController
    .a()
    .whileTrue(Commands.run(() -> armRotation.setMotorVolt(0.1)));
    m_mechanicsController
    .b()
    .whileTrue(Commands.run(() -> armRotation.setMotorVolt(-0.1)));
    m_mechanicsController
    .rightTrigger()
    .whileTrue(Commands.run(() -> armCatch.setMotorRightVolt(0.1)));
    m_mechanicsController
    .leftTrigger()
    .whileTrue(Commands.run(() -> armCatch.setMotorLeftVolt(0.1)));
    m_mechanicsController
    .x()
    .onTrue(new ArmCatchCommand(armCatch, ArmCatchConstants.ArmOperatorMode.ReturnToMedium));
    m_mechanicsController
    .y()
    .onTrue(new ArmCatchCommand(armCatch, ArmCatchConstants.ArmOperatorMode.ReturnToDefault));
    m_mechanicsController
    .start()
    .onTrue(new LiftCommand(lift, LiftOperateMode.Up));
    m_mechanicsController
    .back()
    .onTrue(new LiftCommand(lift, LiftOperateMode.Down));
  }

  public Command getAutonomousCommand() {
    //TODO: create the autonomous command
    pathname = "test1";
    return new PP_TrajectoryCommand(driveTrain,
      PathPlanner.loadPath(pathname,
        new PathConstraints(AutoConstants.MaxSpeedMetersPerSecond,AutoConstants.MaxAccelerationMetersPerSecondSquared),
        false),
      true);
  }

  public static double getDriveSpeedSensitivity() {
    return driveSpeedSensitivity;
  }
  public static double getDriveTurnSensitivity() {
    return driveTurnSensitivity;
  }
}
