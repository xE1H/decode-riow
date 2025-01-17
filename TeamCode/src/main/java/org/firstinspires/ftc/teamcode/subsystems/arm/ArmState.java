package org.firstinspires.ftc.teamcode.subsystems.arm;

import com.arcrobotics.ftclib.command.CommandBase;

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

    public enum State {
        IN_ROBOT,
        INTAKE,
        DEPOSIT,
        SECOND_STAGE_HANG
    }
}