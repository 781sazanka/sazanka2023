package frc.robot;

import java.util.HashMap;
import java.util.Map;

/**
 * @test finished
 */
public class DriveConfig {
    private static final Map<String, DriveConfig> PRESETS = new HashMap<>();
    
    private final double speedSensitivity;
    private final double turnSensitivity;
    
    //set specific status in order to much personal preference
    static {
        PRESETS.put("koken", new DriveConfig(2, 2.9));
        PRESETS.put("person_2", new DriveConfig(1.5, 2));
    }

    public DriveConfig(double speedSensitivity, double turnSensitivity) {
        this.speedSensitivity = speedSensitivity;
        this.turnSensitivity = turnSensitivity;
    }

    //get the specific DriveConfig values from the key value "string"
    public static DriveConfig getPreset(String string) {
        return PRESETS.get(string);
    }

    //get the information of the current driver
    public static DriveConfig getCurrent() {
        // may be bad to get shuffleboard values constantly        
        return PRESETS.getOrDefault(RobotContainer.drivePresetsChooser.getSelected(), PRESETS.get("koken"));
    }

    public double getSpeedSensitivity() {
        return speedSensitivity;
    }

    public double getTurnSensitivity() {
        return turnSensitivity;
    }
}
