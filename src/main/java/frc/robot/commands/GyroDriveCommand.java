// package frc.robot.commands;

// import javax.sql.XADataSource;

// import edu.wpi.first.wpilibj.interfaces.Gyro;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.Constants.DriveConstants;
// import frc.robot.subsystems.ArmCatch;
// import frc.robot.subsystems.NavXGyro;

// /**
//  * @review finished(3/18 9:50)
//  * @test 3/17 going to test
//  */
// public class GyroDriveCommand extends CommandBase {
//   private final NavXGyro gyro;
//   private double[] xyAxises = {0};
//   private boolean[] xyAutoBalanceMode = {false};
//   private boolean autoBalanceXMode = false;
//   private boolean autoBalanceYMode = false;
//   private double pitchAngleRadians = 0;
//   private double rollAngleRadians = 0;

//   private enum XY {
//     X,
//     Y
//   };

//   /**
//    * @param arm
//    * @param setPointInMeters this is not a absolute setpoint, it is a relative
//    *                         setpoint from the robot starts
//    */
//   public GyroDriveCommand(NavXGyro gyro) {
//     this.gyro = gyro;
//     addRequirements(gyro);
//   }

//   @Override
//   public void initialize() {
//     SmartDashboard.putString("GyroDrive Command Start", "moving");
//   }

//   @Override
//   public void execute() {
    
//   }

//   @Override
//   public boolean isFinished() {
//     // TODO: make sure this will return true when the arm reaches the setpoint

//     if (this.arm.isGoal()) {
//       SmartDashboard.putString("ArmCatch State", setPointInMeters + "reached");
//       return true;
//     } else {
//       return false;
//     }
//   }

//   public double[] getXYAxis() {
//     return this.xyAxises;
//   }

//   public boolean[] getXYAutoBalanceMode() {
//     return this.xyAutoBalanceMode;
//   }
//   public double[] GETXYAxis() {
//     // double[] Axises = [0];

//     if (!autoBalanceXMode &&
//       (Math.abs(gyro.getPitchAngleDegrees()) >= 
//         Math.abs(DriveConstants.kOffBalanceAngleThresholdDegrees))) {
//       xyAutoBalanceMode[0] = true;
//     } else if (autoBalanceXMode &&
//       (Math.abs(gyro.getPitchAngleDegrees())) <= 
//         Math.abs(DriveConstants.kOonBalanceAngleThresholdDegrees)) {
//       autoBalanceXMode = false;
//     }
//     if (!autoBalanceYMode &&
//       (Math.abs(gyro.getPitchAngleDegrees()) >= 
//         Math.abs(DriveConstants.kOffBalanceAngleThresholdDegrees))) {
//       autoBalanceYMode = true;
//     } else if (autoBalanceYMode &&
//       (Math.abs(gyro.getPitchAngleDegrees()) <=
//         Math.abs(DriveConstants.kOonBalanceAngleThresholdDegrees))) {
//       autoBalanceYMode = false;
//     }

//     if (autoBalanceXMode) {
//       pitchAngleRadians = gyro.getPitchAngleDegrees() * (Math.PI / 180.0);
//       xyAxises[0] = -Math.sin(pitchAngleRadians);
//     }
//     if (autoBalanceYMode) {
//       rollAngleRadians = gyro.getRollAngleDegrees() * (Math.PI / 180.0);
//       xyAxises[1] = -Math.sin(rollAngleRadians);
//     }
//   }
//   /*
//    * don't need to call end method since it will automatically excute default
//    * command
//    * after isFinished has called
//    */
// }
