/* (C)2022 Max Niederman, Silas Gagnon, and contributors */
package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.math.controller.RamseteController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;

import static frc.robot.Constants.Drivetrain;
import static frc.robot.Constants.AutoConstants;

public class Test_FollowTrajectoryCommand extends SequentialCommandGroup{

    public Test_FollowTrajectoryCommand(DriveTrain drive, PathPlannerTrajectory traj,boolean isFirstpath){
       
        super(
            new InstantCommand(() -> {
                if(isFirstpath){
                    drive.resetOdometry(traj.getInitialPose());
                }
            }),
            new PPRamseteCommand(
                traj,
                drive::getPose,
                new RamseteController(
                        AutoConstants.RAMSETE_B, AutoConstants.RAMSETE_ZETA),
                new SimpleMotorFeedforward(
                        Drivetrain.FEED_FORWARD_KS,
                        Drivetrain.FEED_FORWARD_KV,
                        Drivetrain.FEED_FORWARD_KA),
                Drivetrain.KINEMATICS,
                drive::getWheelSpeeds,
                new PIDController(Drivetrain.DRIVE_VELOCITY_KP, 0, 0),
                new PIDController(Drivetrain.DRIVE_VELOCITY_KP, 0, 0),
                drive::tankDriveVolts,
                true,
                drive),
            new InstantCommand(() -> {
                SmartDashboard.putString("state of the robot", "robot has stopped");
                drive.stop();
                drive.setNeutralMode(NeutralMode.Coast);
            })
        );
    }
}


