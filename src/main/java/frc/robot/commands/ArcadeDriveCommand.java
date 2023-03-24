package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

/**
 *  @review finished(3/18 9:50)
 * @test 3/17 going to test
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
      //TODO; this might be bad for this is overheading

      double speed = -RobotContainer.m_driverController.getLeftY();
      double turn = -RobotContainer.m_driverController.getRightX();

      driveTrain.arcadeDrive(speed / RobotContainer.getDriveSpeedSensitivity(), turn / RobotContainer.getDriveTurnSensitivity());
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
