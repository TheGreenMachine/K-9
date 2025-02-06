package com.team1816.lib.hardware;

import static org.junit.Assert.assertNotNull;

import org.junit.Assert;
import org.junit.Test;

public class RobotYamlTests {

    @Test
    public void defaultYamlTest() {
        loadConfig("default");
    }

    @Test
    public void pidYamlTest() {
        var config = loadConfig("CheezeCurd");
        var drive = config.subsystems.get("drivetrain");
        assertNotNull(drive);
        var pid = drive.pidConfig.get("slot0");
        assertNotNull(pid);
        Assert.assertNotEquals(0.0, pid.kF);
    }

    @Test
    public void cheezeCurdYamlTest() {
        loadConfig("CheezeCurd");
    }

    @Test
    public void zenithYamlTest() {
        loadConfig("zenith");
    }

    private RobotConfiguration loadConfig(String configName) {
        RobotConfiguration config = null;
        try {
            config =
                YamlConfig.loadFrom(
                    this.getClass()
                        .getClassLoader()
                        .getResourceAsStream(configName + ".config.yml")
                );
        } catch (Exception e) {
            e.printStackTrace();
        }
        assertNotNull(config);
        return config;
    }
}
