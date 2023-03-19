/* (C)2022 Max Niederman, Silas Gagnon, and contributors */
package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.math.controller.RamseteController;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;

import static frc.robot.Constants.DriveConstants;
import static frc.robot.Constants.AutoConstants;

public class PP_TrajectoryCommand extends CommandBase {
  private DriveTrain drive;
  private PathPlannerTrajectory traj;
  private boolean isFirstpath;

  public PP_TrajectoryCommand(DriveTrain drive, PathPlannerTrajectory traj, boolean isFirstpath){
      this.drive = drive;
      this.traj = traj;
      this.isFirstpath = isFirstpath;
  }

  SequentialCommandGroup getTrajectoryCommand() {
    return new SequentialCommandGroup(
      new InstantCommand(() -> {
        // Reset odometry for the first path you run during auto
        if(this.isFirstpath){
          this.drive.resetOdometry(traj.getInitialPose());
        }
      }),
      new PPRamseteCommand(
        traj,
        drive::getPose,
        new RamseteController(
          AutoConstants.RAMSETE_B, AutoConstants.RAMSETE_ZETA),
        new SimpleMotorFeedforward(
          DriveConstants.FEED_FORWARD_KS,
          DriveConstants.FEED_FORWARD_KV,
          DriveConstants.FEED_FORWARD_KA),
        DriveConstants.KINEMATICS,
        drive::getWheelSpeeds,
        new PIDController(DriveConstants.DRIVE_VELOCITY_KP, 0, 0),
        new PIDController(DriveConstants.DRIVE_VELOCITY_KP, 0, 0),
        drive::tankDriveVolts,
        true,
        drive),
      new InstantCommand(() -> {
        drive.stop();
        drive.setNeutralMode(NeutralMode.Brake);
      })
  );    
  }
}


