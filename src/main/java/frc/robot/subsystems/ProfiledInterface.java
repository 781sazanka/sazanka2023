package frc.robot.subsystems;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public interface ProfiledInterface {
    void useOutput(double output, TrapezoidProfile.State setpoint);
    double getMeasurement();
    void runWithSetPoint(double setPointInRads);
    double getSetPoint();
    void resetEncoders();
    void resetPositions();
    // void setMotorVolt(double volt, boolean inverted);
    void stop();
    void getConvertedEncoderData();
}
