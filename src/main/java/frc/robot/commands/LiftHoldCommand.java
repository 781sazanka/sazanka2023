package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LiftSubsystem;

public class LiftHoldCommand extends CommandBase {
    private final LiftSubsystem slider;

    public LiftHoldCommand(LiftSubsystem slider) {
        this.slider = slider;
        addRequirements(slider);
    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("Lift Hold SetPointInRads", slider.getSetPointInRads());
    }

    @Override
    public void execute() {
        slider.runWithSetPoint(slider.getSetPointInRads());
    }
}
