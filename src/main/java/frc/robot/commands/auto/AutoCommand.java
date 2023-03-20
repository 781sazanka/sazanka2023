package frc.robot.commands.auto;

import java.util.List;
import java.util.Map;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.RamseteAutoBuilder;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveTrain;

public class AutoCommand {
    static Map<String, Command> eventMap = Map.of(
        "print", new InstantCommand(() -> System.out.println("auto has started")),
        "score", new InstantCommand(() -> System.out.println("auto has scored")).withTimeout(3),
        "take",  new InstantCommand(() -> System.out.println("auto has took cube/ball")).withTimeout(2),
        "secondScore", new InstantCommand(() -> System.out.println("auto has second scored")).withTimeout(2),
        "balance", new InstantCommand(() -> System.out.println("auto has balanced")).withTimeout(2)
    );

    public static Command makeAutoCommand(DriveTrain drive, String pathName) {
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(pathName, AutoConstants.MaxSpeedMetersPerSecond, AutoConstants.MaxAccelerationMetersPerSecondSquared);

        RamseteAutoBuilder autoBuilder = new RamseteAutoBuilder(
            drive::getPose,
            drive::resetOdometry,
            new RamseteController(AutoConstants.RAMSETE_B,AutoConstants.RAMSETE_ZETA),
            DriveConstants.KINEMATICS,
            new SimpleMotorFeedforward(
                DriveConstants.FEED_FORWARD_KS,
                DriveConstants.FEED_FORWARD_KV,
                DriveConstants.FEED_FORWARD_KA),
            drive::getWheelSpeeds,
            new PIDConstants(DriveConstants.DRIVE_VELOCITY_KP, 0, 0),
            drive::tankDriveVolts,
            eventMap,
            true,
            drive
        );

        return autoBuilder.fullAuto(pathGroup);
    }
}
