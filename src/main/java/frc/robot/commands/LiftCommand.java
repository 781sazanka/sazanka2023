package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LiftConstants;
import frc.robot.subsystems.Lift;

/**
 *  @review finished(3/18 9:50)
 * @test 3/17 going to test
 */
public class LiftCommand extends CommandBase {
  private final Lift lift;
  private boolean isUp;
  private boolean isFinished;

  public LiftCommand(Lift lift, boolean isUp) {
    this.lift = lift;
    this.isUp = isUp;
    addRequirements(lift);
  }

  @Override
  public void initialize() {
    if (isUp) {
      SmartDashboard.putString("Lift State", "UP moving");
      lift.setSetPoint(LiftConstants.LiftExtendedPos);
    } else {
      SmartDashboard.putString("Lift State", "DOWN moving");
      lift.setSetPoint(LiftConstants.LiftHorizontalPos);
    }
  }

  @Override
  public void execute() {
    if (isUp) {
      if (lift.getMeasurement() < 0.3) {
        lift.setMotorVolt(0.3);
      } else if(lift.getMeasurement() < 0.6){
        lift.setMotorVolt(0.15);
      } else if(lift.getMeasurement() < 0.9){
        lift.setMotorVolt(0.1);
      } else {
        lift.setMotorVolt(0);
      }
    } else {
      if (lift.getMeasurement() > 0.8) {
        lift.setMotorVolt(-0.05);
      } else if(lift.getMeasurement() > 0.6){
        lift.setMotorVolt(-0.03);
      } else if(lift.getMeasurement() > 0.3){
        lift.setMotorVolt(0.05);
      } else {
        lift.setMotorVolt(0.1);
      }
    }
  }

  @Override
  public boolean isFinished(){
    // TODO: figure out whether isGoal method will work without disabled mode
    if (lift.isGoal()) {
      if (isUp)
        SmartDashboard.putString("Lift State","UP Reached");
      else 
        SmartDashboard.putString("Lift State", "DOWN Reached");
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
