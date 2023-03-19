package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmCatch;

/**
 *  @review finished(3/18 9:50)
 * @test 3/17 going to test
 */
public class ArmCatchCommand extends CommandBase {
  private final ArmCatch arm;
  private final double setPointInMeters;

  /**
   * @param arm
   * @param setPointInMeters this is not a absolute setpoint, it is a relative
   *                         setpoint from the robot starts
   */
  public ArmCatchCommand(ArmCatch arm, double setPointInMeters) {
    this.arm = arm;
    this.setPointInMeters = setPointInMeters;
    addRequirements(arm);
  }

  @Override
  public void initialize() {
    SmartDashboard.putString("ArmCatch State", setPointInMeters + "moving");
    arm.runWithSetPoint(setPointInMeters);
  }

  @Override
  public boolean isFinished() {
    // TODO: make sure this will return true when the arm reaches the setpoint

    if (this.arm.isGoal()) {
      SmartDashboard.putString("ArmCatch State", setPointInMeters + "reached");
      return true;
    } else {
      return false;
    }
  }

  /*
   * don't need to call end method since it will automatically excute default
   * command
   * after isFinished has called
   */
}
