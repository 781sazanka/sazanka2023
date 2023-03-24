// package frc.robot.commands;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj2.command.Commands;
// import frc.robot.Constants.Direction;
// import frc.robot.subsystems.ArmCatch;

// /**
//  *  @review finished(3/18 9:50)
//  * @test 3/17 going to test
//  */
// public class ArmCatchCommand extends CommandBase {
//   private final ArmCatch arm;
//   private final double setPointInMeters;
//   private boolean[] isExecutedOnce = {false,false};
  
//   /**
//    * @param arm
//    * @param setPointInMeters this is not a absolute setpoint, it is a relative
//    *                         setpoint from the robot starts
//    */
//   public ArmCatchCommand(ArmCatch arm, double setPointInMeters) {
//     this.arm = arm;
//     this.setPointInMeters = setPointInMeters;
//     addRequirements(arm);
//   }

//   @Override
//   public void initialize() {
//     arm.parameterIinit();
//     SmartDashboard.putString("ArmCatch State", setPointInMeters + "moving");

//   }

//   @Override
//   public void execute() {
//     if(!arm.isLeftReached() && !arm.isRightReached()) {
//       arm.executeFollow();
//     }
//     else if(arm.isLeftReached() && !(isExecutedOnce[Direction.Left.getCode()])) {
//       Commands.runOnce(() -> arm.executeLeftReached(), arm);
//       isExecutedOnce[Direction.Left.getCode()] = true;
//     }
//     else if(arm.isRightReached() && !(isExecutedOnce[Direction.Right.getCode()])) {
//       Commands.runOnce(() -> arm.executeRightReached(), arm);
//       isExecutedOnce[Direction.Left.getCode()] = true;
//     }
//   }

//   @Override
//   public boolean isFinished() {
//     // TODO: make sure this will return true when the arm reaches the setpoint

//     if (arm.isBothReached()) {
//       SmartDashboard.putString("ArmCatch State", setPointInMeters + "reached");
//       Commands.runOnce(() -> arm.executeBothReached(), arm);
//       return true;
//     } else {
//       return false;
//     }
//   }

//   /*
//    * don't need to call end method since it will automatically excute default
//    * command
//    * after isFinished has called
//    */
// }

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmCatchConstants;
import frc.robot.Constants.ArmCatchConstants.ArmOperatorMode;
import frc.robot.subsystems.ArmCatch;

/**
 *  @review finished(3/18 9:50)
 * @test 3/17 going to test
 */
public class ArmCatchCommand extends CommandBase {
  private final ArmCatch arm;
  private final ArmOperatorMode mode;
  private double setPoint = 0;
  private double currentPoint = 0;
  private boolean[] isExecutedOnce = {false,false};
  private boolean isReachedMedium = false;
  private boolean isReachedDefault = false;
  /**
   * @param arm
   * @param setPointInMeters this is not a absolute setpoint, it is a relative
   *                         setpoint from the robot starts
   */
  public ArmCatchCommand(ArmCatch arm, ArmOperatorMode mode) {
    this.arm = arm;
    this.mode = mode;
    addRequirements(arm);
  }

  @Override
  public void initialize() {
    arm.parameterIinit();

    switch (mode) {
      case ReturnToMedium:
        arm.setSetPoint(ArmCatchConstants.ArmLeftMediumPose, 0.5);
        SmartDashboard.putString("Lift State", "ReturnToMedium moving");
        break;
      case ReturnToDefault:
      arm.setSetPoint(ArmCatchConstants.ArmFarPose, 0.5);
        SmartDashboard.putString("Lift State", "ReturnToDefault moving");
        break;
    }
  }

  @Override
  public void execute() {
    switch (mode) {
      case ReturnToMedium:
        isReachedMedium = arm.executeReturnToMedium();
        break;
      case ReturnToDefault:
        isReachedDefault = arm.executeReturnToDefault();
        break;
    }
  }

  @Override
  public boolean isFinished() {
    // TODO: make sure this will return true when the arm reaches the setpoint
    switch (mode) {
      case ReturnToMedium:
        return isReachedMedium ? true : false;
      case ReturnToDefault:
        return isReachedDefault ? true : false;
      default:
        System.out.println("ERROR :: UndefinedMode for Lift Subsystem\n");
        return true;
    }

  /*
   * don't need to call end method since it will automatically excute default
   * command
   * after isFinished has called
   */
  }
}

