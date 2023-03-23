package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LiftConstants;
import frc.robot.subsystems.Lift;
import frc.robot.Constants.LiftConstants.LiftOperateMode;

/**
 *  @review finished(3/22 23:00)
 *  @mode : Stay is the default mode of Lift
 */
public class LiftCommand extends CommandBase {
  private final Lift lift;
  private LiftOperateMode mode;

  public LiftCommand(Lift lift, LiftOperateMode mode) {
    this.lift = lift;
    this.mode = mode;
    addRequirements(lift);
  }

  @Override
  public void initialize() {
    switch (mode) {
      case Up:
        SmartDashboard.putString("Lift State", "UP moving");
        lift.setSetPoint(LiftConstants.LiftExtendedPos);
        break;
      case Down:
        SmartDashboard.putString("Lift State", "DOWN moving");
        lift.setSetPoint(LiftConstants.LiftHorizontalPos);
        break;
      case Stay:
        SmartDashboard.putString("Lift State", "STAY moving");
        // setPointは現在位置のまま
        break;
    }
  }

  @Override
  public void execute() {
    switch (mode) {
      case Up:
        lift.executeUp();
        break;
      case Down:
        lift.executeDown();
        break;
      case Stay:
        lift.executeStay();
    }
  }

  @Override
  public boolean isFinished() {
    if (lift.isSetPoint()) {
      switch (mode) {
        case Up:
          SmartDashboard.putString("Lift State","UP Reached");
          return true;
        case Down:
          SmartDashboard.putString("Lift State", "DOWN Reached");
          return true;
        case Stay:
          return false;
        default:
          System.out.println("ERROR :: UndefinedMode for Lift Subsystem\n");
          return true;
        }
    } else {
      return false;
    }
  /*
  * don't need to call end method since it will automatically excute default command
  * after isFinished has called
  */
  }
}
