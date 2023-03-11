package frc.robot;

import java.util.HashMap;
import java.util.Map;

public class DriveConfig {
    private static final Map<String, DriveConfig> PRESETS = new HashMap<>();
    
    private final double speedSensitivity;
    private final double turnSensitivity;
    
    //set specific status in order to much personal preference
    static {
        PRESETS.put("koken", new DriveConfig(1, 2.9));
        PRESETS.put("person_2", new DriveConfig(0.7, 2));
    }
    //get the specific DriveConfig values from the key value "string"
    public static DriveConfig getPreset(String string) {
        return PRESETS.get(string);
    }

    //get the information of the current driver
    public static DriveConfig getCurrent() {
        // may be bad to get shuffleboard values constantly

        // decommentout this sentense if you want to have options who drive the robot
        
        // return PRESETS.getOrDefault(RobotContainer.drivePresetsChooser.getSelected(), PRESETS.get("koken"));
        return PRESETS.get("koken");
    }

    public DriveConfig(double speedSensitivity, double turnSensitivity) {
        this.speedSensitivity = speedSensitivity;
        this.turnSensitivity = turnSensitivity;
    }

    public double getSpeedSensitivity() {
        return speedSensitivity;
    }

    public double getTurnSensitivity() {
        return turnSensitivity;
    }
}
