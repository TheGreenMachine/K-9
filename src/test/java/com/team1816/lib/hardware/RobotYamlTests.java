package com.team1816.lib.hardware;

import com.team1816.lib.hardware.factory.YamlConfig;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

public class RobotYamlTests {

    @Test
    public void pidYamlTest() {
        var config = loadConfig("K9");
        var drive = config.subsystems.get("drivetrain");
        assertNotNull(drive);
        var pid = drive.pidConfig.get("slot0");
        assertNotNull(pid);
        assertNotEquals(0.0, pid.kS);
    }

    @Test
    public void K9LoadTest() {
        loadConfig("K9");
    }

    private RobotConfiguration loadConfig(String configName) {
        RobotConfiguration config = null;
        try {
            config =
                YamlConfig.loadFrom(
                    this.getClass()
                        .getClassLoader()
                        .getResourceAsStream("yaml/"+ configName + ".yml")
                );
        } catch (Exception e) {
            e.printStackTrace();
        }
        assertNotNull(config);
        return config;
    }
}
