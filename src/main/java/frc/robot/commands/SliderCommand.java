package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SliderConstants.SliderOperateMode;
import static frc.robot.Constants.SliderConstants.*;
import frc.robot.subsystems.Slider;

/**
 *  @review finished(3/18 9:50)
 * @test 3/17 going to test
 */
public class SliderCommand extends CommandBase {
  private final Slider slider;
  private double setPointInMeters;
  private final SliderOperateMode mode;

  /**
   * @param slider               
   * @param setPointInMeters    this is not a absolute setpoint, it is a relative setpoint from the robot starts
   *                            if this is not CustomSetPoint, write 999 to this argument
   */
  public SliderCommand(Slider slider,SliderOperateMode mode,double setPointInMeters) {
    this.slider = slider;
    this.setPointInMeters = setPointInMeters;
    this.mode = mode;
    addRequirements(slider);
  }

  @Override
  public void initialize() {
    switch (mode) {
      case ExtendAll:
        slider.setSetPoint(SliderLongestInMeters);
        slider.runWithSetPoint(SliderLongestInMeters);
        SmartDashboard.putString("Slider State : ExtendAll", SliderLongestInMeters + "moving");
        break;
      case ShrinkAll:
        slider.setSetPoint(SliderShortestInMeters);
        slider.runWithSetPoint(SliderShortestInMeters);
        SmartDashboard.putString("Slider State : ShrinkAll", SliderShortestInMeters + "moving");
        break;
      case CustomSetPoint:
        slider.setSetPoint(setPointInMeters);
        slider.runWithSetPoint(setPointInMeters);
        SmartDashboard.putString("Slider State : CustomSetPoint", setPointInMeters + "moving");
        break;
      case Stay:
        // only use feedforward control
        // No need to set a new setpoint, it will use last set setpoint to stay at the point
        slider.setSetPoint(slider.getSetPoint());
        SmartDashboard.putString("Slider State : Stay", slider.getSetPoint() + "moving");
        break;
    }
    slider.enable();
  }

  // @Override
  // public void execute() {
  //   switch (mode) {
  //     case ExtendAll:
  //       slider.disable();
  //       slider.executeExtendAll();
  //       break;
  //     case ShrinkAll:
  //       slider.executeShrinkAll();
  //       break;
  //     case CustomSetPoint:
  //       slider.executeCustomSetPoint();
  //       break;
  //     case Stay:
  //       slider.executeStay();
  //       break;
  //   }
  // }

  @Override
  public boolean isFinished() {
    if (slider.isSetPoint()) {
      switch (mode) {
        case ExtendAll:
         SmartDashboard.putString("Slider State : ExtendAll", SliderLongestInMeters + "reached");
         return true;
        case ShrinkAll:
          SmartDashboard.putString("Slider State : ShrinkAll", SliderShortestInMeters + "reached");
          return true;
          case CustomSetPoint:
          SmartDashboard.putString("Slider State : CustomSetPoint", setPointInMeters + "reached");
          return true;
        case Stay:
          SmartDashboard.putString("Slider State : Stay", slider.getSetPoint() + "reached");
          return false;
        default:
          System.out.println("ERROR :: UndefinedMode for Slider Subsystem\n");
          return true;
      }
    } else {
      return false;
    } 
  }

  /*
  * don't need to call end method since it will automatically excute default command
  * after isFinished has called
  */
}
