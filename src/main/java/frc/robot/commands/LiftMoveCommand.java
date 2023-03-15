package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LiftSubsystem;

public class LiftMoveCommand extends CommandBase {
    private final LiftSubsystem slider;
    private final double setPointInRads;

    public LiftMoveCommand(LiftSubsystem slider, double setPointInRads) {
        this.slider = slider;
        this.setPointInRads = setPointInRads;
        addRequirements(slider);
    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("Lift Move SetPointInRads", setPointInRads);
    }
    @Override
    public void execute() {
        slider.runWithSetPoint(setPointInRads);
    }
}
