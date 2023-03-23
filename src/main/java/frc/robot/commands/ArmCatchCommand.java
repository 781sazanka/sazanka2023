package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Direction;
import frc.robot.subsystems.ArmCatch;

/**
 *  @review finished(3/18 9:50)
 * @test 3/17 going to test
 */
public class ArmCatchCommand extends CommandBase {
  private final ArmCatch arm;
  private final double setPointInMeters;
  private boolean[] isExecutedOnce = {false,false};
  
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
    arm.parameterIinit();
    SmartDashboard.putString("ArmCatch State", setPointInMeters + "moving");

  }

  @Override
  public void execute() {
    if(!arm.isLeftReached() && !arm.isRightReached()) {
      arm.executeFollow();
    }
    else if(arm.isLeftReached() && !(isExecutedOnce[Direction.Left.getCode()])) {
      Commands.runOnce(() -> arm.executeLeftReached(), arm);
      isExecutedOnce[Direction.Left.getCode()] = true;
    }
    else if(arm.isRightReached() && !(isExecutedOnce[Direction.Right.getCode()])) {
      Commands.runOnce(() -> arm.executeRightReached(), arm);
      isExecutedOnce[Direction.Left.getCode()] = true;
    }
  }

  @Override
  public boolean isFinished() {
    // TODO: make sure this will return true when the arm reaches the setpoint

    if (arm.isBothReached()) {
      SmartDashboard.putString("ArmCatch State", setPointInMeters + "reached");
      Commands.runOnce(() -> arm.executeBothReached(), arm);
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
