
package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.TestConstants;

public class TestClass {
    static public XboxController controller = new XboxController(0);
    public static class ArmSliderTest{
        private final CANSparkMax arm_motor = new CANSparkMax(TestConstants.ArmSliderRightID, CANSparkMaxLowLevel.MotorType.kBrushless);
        private final CANSparkMax armRotation_motor = new CANSparkMax(TestConstants.ArmRotationID, CANSparkMaxLowLevel.MotorType.kBrushless);
        private RelativeEncoder arm_encoder = arm_motor.getEncoder();
        private RelativeEncoder armRotation_encoder = armRotation_motor.getEncoder();

        public void init(){
            arm_motor.restoreFactoryDefaults();
            armRotation_motor.restoreFactoryDefaults();
            arm_motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
            armRotation_motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
            arm_motor.setSmartCurrentLimit(30);
            armRotation_motor.setSmartCurrentLimit(30);

            arm_motor.setInverted(false);
            armRotation_motor.setInverted(false);

            arm_encoder.setPosition(0);
            armRotation_encoder.setPosition(0);
        }

        public void move_motor(){
            arm_motor.set(controller.getLeftY()/10);
            armRotation_motor.set(controller.getRightY()/10);
            SmartDashboard.putNumber("arm encoder",arm_encoder.getPosition());
            SmartDashboard.putNumber("armRotation encoder", armRotation_encoder.getPosition());
        }
    }
    public static class ArmRotationTest{
        private final CANSparkMax motor = new CANSparkMax(TestConstants.ArmRotationID, CANSparkMaxLowLevel.MotorType.kBrushless);
        private RelativeEncoder encoder = motor.getEncoder();

        public void init(){
            motor.restoreFactoryDefaults();
            motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
            motor.setSmartCurrentLimit(30);
            motor.setInverted(false);
            encoder.setPosition(0);
        } 

        public void move_motor(){
            motor.set(TestClass.controller.getLeftY()/10);
            SmartDashboard.putNumber("encoder value", encoder.getPosition());
        }
    }
}
