package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.DriveConfig;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

/**
 * @test remains command combining
 */
public class ArcadeDriveCommand extends CommandBase {

  private final DriveTrain driveTrain;

  public ArcadeDriveCommand(DriveTrain driveTrain) {
    this.driveTrain = driveTrain;
    addRequirements(driveTrain);
  }

  @Override
  public void initialize() {
    driveTrain.setNeutralMode(NeutralMode.Brake);
  }

  @Override
    public void execute() {
      DriveConfig config = DriveConfig.getCurrent();

      double speed = RobotContainer.m_driverController.getLeftY();
      double turn = RobotContainer.m_driverController.getRightX();

      // TODO: configuring the button bindings so that it can move when the button x() become true
      // driveTrain.arcadeDrive(speed / config.getSpeedSensitivity(), turn / config.getTurnSensitivity());

      RobotContainer.m_driverController
        .x()
        .onTrue(Commands.run(() -> driveTrain.arcadeDrive(speed / config.getSpeedSensitivity(), turn / config.getTurnSensitivity()), driveTrain));
      RobotContainer.m_driverController
        .y()
        .onTrue(Commands.run(() -> driveTrain.stop(), driveTrain));
    }

  @Override
  public boolean isFinished() {
    // TODO: Make this return true when this Command no longer needs to run
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    driveTrain.stop();
  }
}
