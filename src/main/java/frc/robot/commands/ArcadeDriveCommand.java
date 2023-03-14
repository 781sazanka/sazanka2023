/* (C)2022 Max Niederman, Silas Gagnon, and contributors */
package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.DriveConfig;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

public class ArcadeDriveCommand extends CommandBase {

    private final DriveTrain driveTrain;

    public ArcadeDriveCommand(DriveTrain driveTrain) {
        this.driveTrain = driveTrain;
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        driveTrain.setNeutralMode(NeutralMode.Coast);
    }

    @Override
    public void execute() {
        DriveConfig config = DriveConfig.getCurrent();
        // TODO: considering the usage of controller
        double speed = RobotContainer.m_driverController.getLeftY();
        double turn = RobotContainer.m_driverController.getRightX();

        // TODO: changing the button
        RobotContainer.m_driverController.x()
            .onTrue(new InstantCommand(() -> {driveTrain.stop();}))
            .onFalse(new InstantCommand(() -> {driveTrain.arcadeDrive(speed / config.getSpeedSensitivity(), turn / config.getTurnSensitivity());}));
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.stop();
    }
}
