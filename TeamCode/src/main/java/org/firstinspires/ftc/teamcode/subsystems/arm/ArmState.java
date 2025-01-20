package org.firstinspires.ftc.teamcode.subsystems.arm;

import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration;

public class ArmState {
    private static State currentState = State.IN_ROBOT;
    private static boolean moving = false;
    private static long lastMoved = 0;

    public static State get() {
        return currentState;
    }

    public static void set(State stateValue) {
        currentState = stateValue;
        moving = false;
    }

    public static boolean isMoving() {
        return System.currentTimeMillis() - lastMoved < ArmSlideConfiguration.ERROR_TIMEOUT_MILLIS && moving;
    }

    public static void setMoving(boolean movingValue) {
        moving = movingValue;
        if (movingValue) {
            lastMoved = System.currentTimeMillis();
        }
    }

    public static boolean isCurrentState(State state) {
        return currentState == state;
    }

    public static boolean isCurrentState(State ...states) {
        for (State state : states) {
            if (currentState == state) {
                return true;
            }
        }
        return false;
    }

    public static void resetAll() {
        currentState = State.IN_ROBOT;
        moving = false;
        lastMoved = 0;
    }

    public enum State {
        IN_ROBOT,
        INTAKE_SAMPLE,
        INTAKE_SPECIMEN,
        SCORE_SAMPLE_LOW,
        SCORE_SAMPLE_HIGH,
        PREPARE_SPECIMEN_HIGH,
        SCORE_SPECIMEN_HIGH,
        PREPARE_SPECIMEN_LOW,
        SCORE_SPECIMEN_LOW,
        SECOND_STAGE_HANG
    }
}