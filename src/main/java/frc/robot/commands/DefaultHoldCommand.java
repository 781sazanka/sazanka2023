package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmCatch;
import frc.robot.subsystems.ArmRotation;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Slider;

/**
 * @review finished(3/18 10:00)
 * @test 3/18
 * this is a default command
 */
public class DefaultHoldCommand extends CommandBase {
    private final Lift lift;
    private final ArmRotation armRotation;
    private final ArmCatch armCatchLeft;
    private final ArmCatch armCatchRight;
    private final Slider slider;

    public DefaultHoldCommand(Lift lift, ArmRotation armRotation, ArmCatch armCatchLeft,ArmCatch armCatchRight, Slider slider) {
        this.lift = lift;
        this.armRotation = armRotation;
        this.armCatchLeft = armCatchLeft;
        this.armCatchRight = armCatchRight;
        this.slider = slider;
        addRequirements(lift,armRotation,armCatchLeft,armCatchRight,slider);
    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("Hold Lift [rad]", lift.getSetPoint());
        SmartDashboard.putNumber(" Hold ArmRotation [rad]", armRotation.getSetPoint());
        SmartDashboard.putNumber("Hold armCatchLeft [m]", armCatchLeft.getSetPoint());
        SmartDashboard.putNumber("Hold armCatchRight [m]", armCatchRight.getSetPoint());
        SmartDashboard.putNumber("Hold slider [m]", slider.getSetPoint());

        lift.runWithSetPoint(lift.getSetPoint());
        armRotation.runWithSetPoint(armRotation.getSetPoint());
        armCatchLeft.runWithSetPoint(armCatchLeft.getSetPoint());
        armCatchRight.runWithSetPoint(armCatchRight.getSetPoint());
        slider.runWithSetPoint(slider.getSetPoint());
    }
}
