package org.firstinspires.ftc.teamcode.subsystems.arm;

public class ArmState {
    private static State currentState = State.IN_ROBOT;

    public static State get() {
        return currentState;
    }

    public static void set(State stateValue) {
        currentState = stateValue;
    }

    public enum State {
        IN_ROBOT,
        INTAKE,
        DEPOSIT,
        SECOND_STAGE_HANG
    }
}