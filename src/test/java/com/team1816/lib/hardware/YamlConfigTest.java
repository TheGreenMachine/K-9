package com.team1816.lib.hardware;

import org.junit.Test;
import org.junit.runner.RunWith;
import org.junit.runners.JUnit4;

import static org.junit.Assert.assertTrue;

@RunWith(JUnit4.class)
public class YamlConfigTest {
    @Test
    public void test() {
        YamlConfig config = YamlConfig.loadFrom(getClass().getClassLoader().getResourceAsStream("default.config.yml"));
        System.out.println(config);
        assertTrue("Verbose is not true for default config", config.getBoolean("verbose"));
    }
}
