package com.team1816.lib;

import com.team1816.lib.events.PubSubHandler;
import com.team1816.lib.hardware.factory.RobotFactory;
import com.team1816.lib.subsystems.ITestableSubsystem;
import com.team1816.lib.util.GreenLogger;

import java.util.Collection;
import java.util.HashMap;

// Used to instantiate singletons.  All subsystems should have a single instance
public class Singleton {

    private static HashMap<Class<?>, Object> instances = null;
    private static HashMap<Class<?>, ITestableSubsystem> subSystems = null;
    public static RobotFactory factory = null;
    public static PubSubHandler pubsub = null;

    // this is used to reset all instances should only be called in tests or first use.
    public static void Reset(){
        instances = new HashMap<>();
        subSystems = new HashMap<>();
        factory = Singleton.get(RobotFactory.class);
        pubsub = Singleton.get(PubSubHandler.class);
    }

    public static Collection<ITestableSubsystem> getSubSystems(){
        if(instances == null) Reset();
        return subSystems.values();
    }

    public static void registerMock(Object obj){
        var clazz = obj.getClass();
        // if we are mocking the robot factory we need to update the static reference
        if(clazz.getName() == factory.getClass().getName()){
            factory = (RobotFactory)obj;
        }
        instances.put(clazz, obj);
    }

    public static <T> T get(Class<T> type) {
        if(instances == null) Reset();
        if (!instances.containsKey(type)) {
            try {
                instances.put(type, type.getDeclaredConstructor().newInstance());
            } catch (Exception e) {
                GreenLogger.log(e);
                return null;
            }
        }
        return type.cast(instances.get(type));
    }

    public static <T extends ITestableSubsystem> T CreateSubSystem(Class<T> type) {
        if(instances == null) Reset();
        if (subSystems.containsKey(type)) {
            throw new RuntimeException("Subsystem " + type.getName() + " already created you can't have more than one instance");
        }
        try {
            var system = type.getDeclaredConstructor().newInstance();
            subSystems.put(type, system);
            return system;
        } catch (Exception e) {
            var msg = e.getMessage();
            if (e.getCause() != null) msg = e.getCause().getMessage();
            throw new RuntimeException(msg);
        }
    }
}

