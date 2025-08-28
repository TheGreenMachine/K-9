package com.team1816.lib.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface ITestableSubsystem extends Subsystem {
    // override  this method to test the subsystem
    default Command TestSubsystem(){
         return null;
     }
}
