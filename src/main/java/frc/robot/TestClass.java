
package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.*;
import frc.robot.commands.LiftCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Slider;

public class TestClass {
  static public XboxController controller = new XboxController(0);
  public static class ArmCatchTest{
    private final CANSparkMax left_motor = new CANSparkMax(TestConstants.ArmCatchLeftID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax right_motor = new CANSparkMax(TestConstants.ArmCatchRightID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private RelativeEncoder left_encoder = left_motor.getEncoder();
    private RelativeEncoder right_encoder = right_motor.getEncoder();

    public void init(){
      left_motor.restoreFactoryDefaults();
      right_motor.restoreFactoryDefaults();
      left_motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
      right_motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
      left_motor.setSmartCurrentLimit(30);
      right_motor.setSmartCurrentLimit(30);
      //TODO: checking if should inverted
      left_motor.setInverted(false);
      right_motor.setInverted(false);
      //TODO: make sure to invert one encoder if needed
      left_encoder.setPositionConversionFactor(ArmCatchConstants.kEncoderDistancePerPulse);
      left_encoder.setVelocityConversionFactor(ArmCatchConstants.kEncoderDistancePerPulse);
      left_encoder.setPositionConversionFactor(ArmCatchConstants.kEncoderDistancePerPulse);
      left_encoder.setVelocityConversionFactor(ArmCatchConstants.kEncoderDistancePerPulse);
        
      left_encoder.setPosition(ArmCatchConstants.ArmFarPose);
      right_encoder.setPosition(ArmCatchConstants.ArmFarPose);
    }

    public void move_motor(){
      left_motor.set(controller.getLeftY()/10);
      right_motor.set(controller.getRightY()/10);
    }

    public void getRightEncoder() {
      SmartDashboard.putNumber("Right Encoder Pos[m]", right_encoder.getPosition());
      SmartDashboard.putNumber("Right Encoder Vel[m/s]", right_encoder.getVelocity());
      SmartDashboard.putNumber("Left Encoder Pos[m]", left_encoder.getPosition());
      SmartDashboard.putNumber("Left Encoder Vel[m/s]", left_encoder.getVelocity());
    }
  }
  public static class ArmRotationTest{
    private final CANSparkMax motor = new CANSparkMax(TestConstants.ArmRotationID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private RelativeEncoder encoder = motor.getEncoder();

    public void init(){
      motor.restoreFactoryDefaults();
      motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
      motor.setSmartCurrentLimit(30);
      //TODO: make sure to rotate CW as the value is positive
      motor.setInverted(false);
      //TODO: make sure that encoder data will increase as it rotates CW
      encoder.setPositionConversionFactor(Units.rotationsToRadians(1) * ArmRotationConstants.ArmGearRatio);
      encoder.setVelocityConversionFactor(Units.rotationsToRadians(1) * ArmRotationConstants.ArmGearRatio);
      
      encoder.setPosition(ArmRotationConstants.ArmButtomRads);
    } 

    public void move_motor(){
      motor.set(TestClass.controller.getLeftY()/10);
    }

    public void getEncoder() {
      SmartDashboard.putNumber("ArmRotation Encoder Pos[rad]", encoder.getPosition());
      SmartDashboard.putNumber("ArmRotation Encoder Vel[rad/s]", encoder.getVelocity());
    }
  }
  public static class SliderTest {
    private final CANSparkMax left_motor = new CANSparkMax(SliderConstants.LeftMotorID,
    CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax right_motor = new CANSparkMax(SliderConstants.RightMotorID,
      CANSparkMaxLowLevel.MotorType.kBrushless);
    private RelativeEncoder encoder = left_motor.getEncoder();

    public void init() {
      left_motor.restoreFactoryDefaults();
      right_motor.restoreFactoryDefaults();
      left_motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
      right_motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
      left_motor.setSmartCurrentLimit(40);
      right_motor.setSmartCurrentLimit(40);
      //TODO: figure out whether should invert this
      right_motor.follow(left_motor, false);
      left_motor.setInverted(false);
      //TODO: make sure encoder value will increase as it extends longer
      encoder.setPositionConversionFactor(SliderConstants.kEncoderDistancePerPulse);
      encoder.setVelocityConversionFactor(SliderConstants.kEncoderDistancePerPulse);

      encoder.setPosition(SliderConstants.SliderShortestInMeters);
    }

    public void move_motor() {
      left_motor.set(TestClass.controller.getLeftY()/10);
    }

    public void getEncoder() {
      SmartDashboard.putNumber("Slider Encoder Pos[m]", encoder.getPosition());
      SmartDashboard.putNumber("Slider Encoder Vel[m/s]", encoder.getVelocity());
    }
  }

  /**
   * @test finished / I can use this parameters as a command
   * @attention it is better to have a feedforward control
   */
  public static class LiftTest {
    private final Lift lift = new Lift(true);
    
    public void init() {
      lift.setSetPoint(LiftConstants.LiftExtendedPos);
    }

    public void move_motor() {
      if (lift.getMeasurement() < 0.3) {
        lift.setMotorVolt(0.3);
      } else if(lift.getMeasurement() < 0.6){
        lift.setMotorVolt(0.15);
      } else if(lift.getMeasurement() < LiftConstants.LiftExtendedPos-0.3){
        lift.setMotorVolt(0.1);
      } else {
        lift.stop();
      }
    }

    public void getEncoder() {
      SmartDashboard.putNumber("setPoint", lift.getSetPoint());
      lift.getConvertedEncoderData();
    }
  }

  public static class DriveTest {
    private DriveTrain drive = new DriveTrain();

    public void drive() {
      drive.tankDrive(-TestClass.controller.getLeftY()/3, -TestClass.controller.getRightY()/3);
      SmartDashboard.putNumber("Controller Left", -TestClass.controller.getLeftY());
      SmartDashboard.putNumber("Right X", -TestClass.controller.getRightY());
    }
    public void display() {
      drive.displayData();
    }
  }

  public static class SimpleMotorTest {
    private final CANSparkMax motor = new CANSparkMax(6, CANSparkMaxLowLevel.MotorType.kBrushless);
    private RelativeEncoder encoder = motor.getEncoder();

    public void init() {
      motor.restoreFactoryDefaults();
      motor.setIdleMode(CANSparkMax.IdleMode.kCoast);
      motor.setInverted(false);

      encoder.setPositionConversionFactor(SliderConstants.kEncoderDistancePerPulse);
      encoder.setVelocityConversionFactor(SliderConstants.kEncoderDistancePerPulse);
      encoder.setPosition(0);
    }

    public void move_motor() {
      if ((encoder.getPosition() <= SliderConstants.SliderLongestInMeters) && (encoder.getPosition() >= SliderConstants.SliderShortestInMeters)) {
        motor.set(-TestClass.controller.getLeftY()/7.0);
      } else {
        stop();
      }
      SmartDashboard.putNumber("controller value", -TestClass.controller.getLeftY());
    }

    public void stop() {
      motor.set(0);
    }

    public void displayData() {
      SmartDashboard.putNumber("Lift Encoder Pos[m]", encoder.getPosition());
      SmartDashboard.putNumber("Lift Encoder Vel[m/s]", encoder.getVelocity());
    }
  }

  public static class LiftSubsystemTest {
    Lift lift = new Lift(true);

    public void init() {
      lift.setSetPoint(LiftConstants.LiftExtendedPos);
      // lift.setSetPoint(LiftConstants.LiftHorizontalPos);
    }
    public void putData() {
      lift.getConvertedEncoderData();
      SmartDashboard.putNumber("getMeasurement [rad]", lift.getMeasurement());
      SmartDashboard.putNumber("getSetPoint", lift.getSetPoint());
      SmartDashboard.putBoolean("isGoal", lift.isSetPoint());
    }
  }

  public static class LiftCommandTest {
    Lift lift = new Lift(true);

    public void init() {
      lift.resetEncoders();
      lift.resetPositions();
    }

    public void excuteUpCommand() {
      SmartDashboard.putData("Lift UP Command", new LiftCommand(lift, true));
    }
    public void excuteDownCommand() {
      SmartDashboard.putData("Lift DOWN Command", new LiftCommand(lift, false));
    }
    public void excuteLiftHoldCommand() {
      SmartDashboard.putData("Lift Hold Command", new LiftHoldCommand(lift));
    }
    public void putData() {
      lift.getConvertedEncoderData();
      SmartDashboard.putBoolean("isGoal", lift.isSetPoint());
      SmartDashboard.putNumber("getSetPoint", lift.getSetPoint());
      SmartDashboard.putNumber("getMeasurement", lift.getMeasurement());
    }

    public static class LiftHoldCommand extends CommandBase {
      private Lift lift;
      public LiftHoldCommand(Lift lift){
        addRequirements(lift);
      }
      @Override
      public void initialize() {
        this.lift.runWithSetPoint(this.lift.getSetPoint());
      }
    }
  }
}
