/* (C)2022 Max Niederman, Silas Gagnon, and contributors */
package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj2.command.CommandBase;
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
        double speed = RobotContainer.controller.getLeftY();
        double turn = RobotContainer.controller.getRightX();

        // TODO: parameter "button" needs to be considered
        if(RobotContainer.controller.getRawButton(3)) {
            driveTrain.stop();
        }
        else {
            driveTrain.arcadeDrive(speed / config.getSpeedSensitivity(), turn / config.getTurnSensitivity());
        }
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
