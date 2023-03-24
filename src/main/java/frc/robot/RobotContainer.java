// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.*;
import frc.robot.TestClass.LiftTest;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import java.lang.ModuleLayer.Controller;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

public class RobotContainer {
  // Shuffleboard
  public static final ShuffleboardTab driveSettingsTab = Shuffleboard.getTab("Drive Settings");
  public static final ShuffleboardTab driveTab = Shuffleboard.getTab("DriveTrain");
  public static final ShuffleboardTab defaultCommandTab = Shuffleboard.getTab("defaulCommand State");
  public static SendableChooser<String> drivePresetsChooser;  // choose who drive
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
  // private static final ArmCatch armCatchLeft = new ArmCatch(true, true);
  // private static final ArmCatch armCatchRight = new ArmCatch(false, true);
  // private static final Slider slider = new Slider(true);
  // private static final ArmRotation testArm = new ArmRotation(true);

  // commands
  private static final ArcadeDriveCommand ARCADE_DRIVE = 
    new ArcadeDriveCommand(driveTrain);
  // private static final DefaultHoldCommand HOLD = 
  //   new DefaultHoldCommand(liftArm, rotationArm,armCatchLeft,armCatchRight,slider);

  private static String pathname;

  public RobotContainer() {
    configureBindings();

    // default commands settings
    driveTrain.setDefaultCommand(ARCADE_DRIVE);
    // lift.setDefaultCommand(new LiftCommand(lift, LiftConstants.LiftOperateMode.Stay));
    // rotationArm.setDefaultCommand(HOLD);
    // armCatchLeft.setDefaultCommand(HOLD);
    // armCatchRight.setDefaultCommand(HOLD);
    // slider.setDefaultCommand(HOLD);

    // driver settings
    drivePresetsChooser = new SendableChooser<String>();
    drivePresetsChooser.addOption("Koken", "koken");
    drivePresetsChooser.addOption("Person_2", "person_2");
    driveSettingsTab.add("Drive Presets", drivePresetsChooser)
      .withWidget(BuiltInWidgets.kComboBoxChooser);
  }

  private void configureBindings() {
    m_driverController
    .a().debounce(0.5)
    .onTrue(new InstantCommand(() -> {driveSpeedSensitivity++; driveTurnSensitivity+=0.5;}));
    m_driverController
    .b().debounce(0.5)
    .onTrue(new InstantCommand(() -> {driveTurnSensitivity--; driveTurnSensitivity-=0.5;}));

    // m_mechanicsController
    // .x()
    // .onTrue(new LiftCommand(lift, LiftConstants.LiftOperateMode.Up));
    // m_mechanicsController
    // .y()
    // .onTrue(new LiftCommand(lift, LiftConstants.LiftOperateMode.Down));
    m_mechanicsController
    .a()
    .onTrue(new ArmRotationCommand(armRotation, ArmRotationConstants.ArmForwardVertRads));
    m_mechanicsController
    .b()
    .onTrue(new ArmRotationCommand(armRotation, ArmRotationConstants.ArmBackwardVertRads));

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
