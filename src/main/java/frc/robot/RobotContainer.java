// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//TODO: find ways to define motor's NeutralMode here
package frc.robot;

import frc.robot.Constants.*;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static final ShuffleboardTab driveSettings = Shuffleboard.getTab("Drive Settings");
  public static final ShuffleboardTab drivetrain = Shuffleboard.getTab("DriveTrain");
  public static SendableChooser<String> drivePresetsChooser;  // choose who drive
  public static Field2d field = new Field2d();                // in case to view the current place of robot

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public static final CommandXboxController m_driverController =
    new CommandXboxController(OperatorConstants.DriverControllerPort);
  public static final CommandPS4Controller m_mechanicsController =
    new CommandPS4Controller(OperatorConstants.MechanicsControllerPort);

  private static final DriveTrain driveTrain = new DriveTrain();
  // private static final LiftSubsystem liftArm = new LiftSubsystem(true);
  private static final ArmRotationSubsystem rotationArm = new ArmRotationSubsystem();

  private static final ArcadeDriveCommand ARCADE_DRIVE = 
    new ArcadeDriveCommand(driveTrain);
  // private static final LiftHoldCommand LIFT_HOLD =
  //   new LiftHoldCommand(liftArm);

  private static String pathname;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    // driveTrain.setDefaultCommand(ARCADE_DRIVE);
    // liftArm.setDefaultCommand(LIFT_HOLD);

    // driver settings
    drivePresetsChooser = new SendableChooser<String>();
    drivePresetsChooser.addOption("Koken", "koken");
    drivePresetsChooser.addOption("Person_2", "person_2");
    driveSettings.add("Drive Presets", drivePresetsChooser)
      .withWidget(BuiltInWidgets.kComboBoxChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // move the arm to the position to put cones and cubes when 'A' button is pressed
    // m_mechanicsController
    //   .L1()
    //   .onTrue(
    //     Commands.runOnce(() -> {
    //       liftArm.setGoal(LiftConstants.LiftGoalPositionRad);
    //       liftArm.enable();
    //     }, liftArm)
    //   );

    // // move the arm to neutral position when 'B' button is pressed
    // m_mechanicsController
    //   .L2()
    //   .onTrue(
    //     Commands.runOnce(() -> {
    //       liftArm.setGoal(LiftConstants.LiftOffsetRads);
    //       liftArm.enable();
    //     }, liftArm)
    //   );
    
    // m_mechanicsController
    //   .L3()
    //   .onTrue(
    //     Commands.run(() -> {
    //       liftArm.setMotorVolt(m_mechanicsController.getLeftY(), false);
    //     }, liftArm)
    //   );
    
    // m_mechanicsController
    // .L3()
    // .onTrue(
    //   Commands.run(() -> {
    //     rotationArm.setMotorVolt(m_mechanicsController.getRightY(), false);
    //   }, rotationArm)
    // );

    // // disable the arm controller when Y is pressed
    // m_mechanicsController.y().onTrue(Commands.runOnce(liftArm::disable));
    
    // Drive at half speed when the bumper is held
    // m_driverController
    // .rightBumper()
    // .onTrue(Commands.runOnce(() -> driveTrain.setMaxOutput(0.5)))
    // .onFalse(Commands.runOnce(() -> driveTrain.setMaxOutput(1.0)));
    
  //   m_driverController.a()
  //   .onTrue(Commands.runOnce(() -> new LiftMoveCommand(liftArm, 2)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //TODO: create the autonomous command
    pathname = "test1";
    return new PP_TrajectoryCommand(driveTrain,
      PathPlanner.loadPath(pathname,
        new PathConstraints(AutoConstants.MaxSpeedMetersPerSecond,AutoConstants.MaxAccelerationMetersPerSecondSquared),
        true),
      true);
    // return new SequentialCommandGroup(
    //   new WaitCommand(5),
    //   new FollowTrajectoryCommand(driveTrain, PathPlanner.loadPath("example path", 
    //     new PathConstraints(AutoConstants.MaxSpeedMetersPerSecond, AutoConstants.MaxAccelerationMetersPerSecondSquared)))
    // );
  }
}
