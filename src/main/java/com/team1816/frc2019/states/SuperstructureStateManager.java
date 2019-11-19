package com.team1816.frc2019.states;


import com.team1816.frc2019.subsystems.CargoShooter;

public class SuperstructureStateManager {
    public enum SubsystemState {
        RESTING_POSITION,
        MOVING_TO_POSITION,
        MANUAL
    }

    private SubsystemState systemState = SubsystemState.RESTING_POSITION;

    private CargoShooter.ArmPosition ROCKET_POSITION = CargoShooter.ArmPosition.ROCKET;
    private CargoShooter.ArmPosition CARGO_POSITION = CargoShooter.ArmPosition.UP;
    private CargoShooter.ArmPosition INTAKE_POSITION = CargoShooter.ArmPosition.DOWN;


    //TODO: Set cargo collector and shooter positions as one state
    // Superstructure class should update the "state" of the cargo collector and shooter (looking at the subsystems holistically)


    public SubsystemState getSystemState() {
        return systemState;
    }



}



