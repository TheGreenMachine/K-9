package com.team1816.lib;

import com.team1816.lib.subsystems.GreenSubsystem;
import com.team1816.lib.util.GreenLogger;

import java.util.HashMap;

// Used to instantiate singletons.  All subsystems should have a single instance
public class Singleton {

    private static final HashMap<Class<?>, Object> instances = new HashMap<>();
    public static final HashMap<Class<?>, Object> subSystems = new HashMap<>();

    public static <T> T get(Class<T> type) {
        if(!instances.containsKey(type)) {
            try {
                instances.put(type,type.getDeclaredConstructor().newInstance());
            } catch (Exception e) {
                GreenLogger.log(e);
                return null;
            }
        }
        return type.cast(instances.get(type));
    }

    public static <T extends GreenSubsystem> void CreateSubSystem(Class<T> type) {
        if(subSystems.containsKey(type)) {
            throw new RuntimeException("Subsystem " + type.getName() + " already created you can't have more than one instance");
        }
        try {
            var system = type.getDeclaredConstructor().newInstance();
            subSystems.put(type,system);
        } catch (Exception e) {
            var msg = e.getMessage();
            if(e.getCause() != null)  msg = e.getCause().getMessage();
            GreenLogger.log(msg);
            throw new RuntimeException(msg);
        }
    }
}

