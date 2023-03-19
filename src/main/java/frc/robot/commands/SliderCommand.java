package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Slider;

/**
 *  @review finished(3/18 9:50)
 * @test 3/17 going to test
 */
public class SliderCommand extends CommandBase {
  private final Slider slider;
  private final double setPointInMeters;

  /**
   * @param slider               
   * @param setPointInMeters    this is not a absolute setpoint, it is a relative setpoint from the robot starts
   */
  public SliderCommand(Slider slider, double setPointInMeters) {
    this.slider = slider;
    this.setPointInMeters = setPointInMeters;
    addRequirements(slider);
  }

  @Override
  public void initialize() {
    SmartDashboard.putString("Slider State", setPointInMeters + "moving");
    slider.runWithSetPoint(setPointInMeters);
  }

  @Override
  public boolean isFinished() {
    if (slider.isGoal()) {
      SmartDashboard.putString("Slider State", setPointInMeters + "reached");
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
