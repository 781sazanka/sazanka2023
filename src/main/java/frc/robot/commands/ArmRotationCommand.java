package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmRotation;

/**
 *  @review finished(3/22 23:45)
 */
public class ArmRotationCommand extends CommandBase {
  private final ArmRotation arm;
  private final double setPointInRads;
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
    SmartDashboard.putString("Arm State", setPointInRads + " moving");
    arm.setSetPoint(setPointInRads);
  }
  
  @Override 
  public void execute() {
    arm.runWithSetPoint(setPointInRads);

    arm.getConvertedEncoderData();
    SmartDashboard.putNumber("getMeasurement [rad]", arm.getMeasurement());
    SmartDashboard.putNumber("getSetPoint [rad]", arm.getSetPoint());
  }
  @Override
  public boolean isFinished() {
    if (arm.checkAtSetPoint()) {
      return true;
    } else {
      return false;
    }
  }
  @Override
  public void end(boolean interrupted) {
    if(interrupted) {
      SmartDashboard.putString("Arm State", setPointInRads + " reached");
    } else {
      SmartDashboard.putString("Arm State", "GET INTERRUPTED");
    }
    arm.stop();

  }

  /*
  * don't need to call end method since it will automatically excute default command
  * after isFinished has called
  */
}
