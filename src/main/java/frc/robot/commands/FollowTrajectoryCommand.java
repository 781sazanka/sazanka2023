/* (C)2022 Max Niederman, Silas Gagnon, and contributors */
package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.math.controller.RamseteController;

import static frc.robot.Constants.Drivetrain;
import static frc.robot.Constants.AutoConstants;

public class FollowTrajectoryCommand extends RamseteCommand {
    private final DriveTrain driveTrain;
    private final Trajectory trajectory;

    private boolean resetOdometry = true;

    public FollowTrajectoryCommand(
            DriveTrain driveTrain,
            Trajectory trajectory) {
        //the constructor of ramsetecommand
        super(
                trajectory,
                driveTrain::getPose,
                new RamseteController(
                        AutoConstants.RAMSETE_B, AutoConstants.RAMSETE_ZETA),
                new SimpleMotorFeedforward(
                        Drivetrain.FEED_FORWARD_KS,
                        Drivetrain.FEED_FORWARD_KV,
                        Drivetrain.FEED_FORWARD_KA),
                Drivetrain.KINEMATICS,
                driveTrain::getWheelSpeeds,
                new PIDController(Drivetrain.DRIVE_VELOCITY_KP, 0, 0),
                new PIDController(Drivetrain.DRIVE_VELOCITY_KP, 0, 0),
                driveTrain::tankDriveVolts,
                driveTrain);

        this.trajectory = trajectory;

        this.driveTrain = driveTrain;
    }

    public FollowTrajectoryCommand(DriveTrain driveTrain, Trajectory trajectory, boolean resetOdometry) {
        this(driveTrain, trajectory);
        this.resetOdometry = resetOdometry;

        driveTrain.setNeutralMode(NeutralMode.Brake);
    }

    // public void addPositionedCommand(PositionedCommand<Command> positionedCommand) {
    //     positionedCommands.add(positionedCommand);
    // }

    @Override
    public void initialize() {
        if(resetOdometry) {
            driveTrain.resetOdometry(trajectory.getInitialPose());
        }
        super.initialize();

        // try {
        //     Files.writeString(Paths.get("/home/lvuser/log.txt"), "", StandardOpenOption.CREATE);
        // } catch (IOException e) {
        //     DriverStation.reportError("unable to create logfile " + e, true);
        // }
    }

    // @Override
    // public void execute() {
    //     super.execute();

    //     Pose2d pose = driveTrain.getPose();

    //     for (PositionedCommand<Command> command : positionedCommands) {
    //         command.executeAt(pose);
    //     }
    // }

    @Override
    public void end(boolean interrupted) {
        driveTrain.stop();
        driveTrain.setNeutralMode(NeutralMode.Coast);
        //set the motor output to zero if interrupted
        super.end(interrupted);
    }

    // public class PositionedCommand<C extends Command> {
    //     private final C command;
    //     private final Pose2d start;
    //     private final Pose2d end;
    //     private final double tolerance;
    //     private boolean running = false;

    //     public PositionedCommand(C command, Pose2d start, Pose2d end, double tolerance) {
    //         this.command = command;
    //         this.start = start;
    //         this.end = end;
    //         this.tolerance = tolerance;
    //     }

    //     public PositionedCommand(C command, Pose2d start, Pose2d end) {
    //         this(command, start, end, 0.0);
    //     }

    //     public void executeAt(Pose2d pose) {
    //         if (transformLength(start.minus(pose)) < tolerance) {
    //             running = true;
    //             command.initialize();
    //         }

    //         if (transformLength(end.minus(pose)) < tolerance) {
    //             running = false;
    //             command.end(true);
    //         }

    //         if (running) {
    //             command.execute();
    //         }
    //     }

    //     double transformLength(Transform2d transform) {
    //         return Math.sqrt(
    //                 Math.pow(transform.getTranslation().getX(), 2)
    //                         + Math.pow(transform.getTranslation().getY(), 2));
    //     }
    // }
}
