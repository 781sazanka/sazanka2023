package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmRotation;

/**
 *  @review finished(3/18 9:50)
 * @test 3/17 going to test
 */
public class ArmRotationCommand extends CommandBase {
  private final ArmRotation arm;
  private final double setPointInRads;
  private final Timer timer = new Timer();
  /**
   * @param arm               
   * @param setPointInRads    this is not a absolute setpoint, it is a relative setpoint from the robot starts
   */
  public ArmRotationCommand(ArmRotation arm, double setPointInRads) {
    this.arm = arm;
    this.setPointInRads = setPointInRads;
    addRequirements(arm);
  }

  //TODO: figure out whether should put runWithSetPoint method in excute or initialize
  @Override
  public void initialize() {
    SmartDashboard.putString("Arm State", setPointInRads + "moving");
    arm.runWithSetPoint(setPointInRads);
    timer.start();
  }
  
  @Override
  public boolean isFinished() {
    // TODO: make sure this will return true when the arm reaches the setpoint
    // TODO; test whether the timer will work as a limit
    if (arm.isGoal() || timer.hasElapsed(10)) {
      SmartDashboard.putString("Arm State", setPointInRads + "reached");
      return true;
    } else {
      return false;
    }
  }

  /*
  * don't need to call end method since it will automatically excute default command
  * after isFinished has called
  */
}
