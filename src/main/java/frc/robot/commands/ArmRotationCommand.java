package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmRotationSubsystem;

public class ArmRotationCommand extends CommandBase {
    private final ArmRotationSubsystem slider;
    private final double setPointInRads;

    public ArmRotationCommand(ArmRotationSubsystem slider, double setPointInRads) {
        this.slider = slider;
        this.setPointInRads = setPointInRads;
        addRequirements(slider);
    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("Arm Move SetPointInRads", setPointInRads);
    }
    @Override
    public void execute() {
        slider.runWithSetPoint(setPointInRads);
    }
}
