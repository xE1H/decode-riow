package org.firstinspires.ftc.teamcode.helpers.controls;

import com.arcrobotics.ftclib.gamepad.GamepadEx;

/**
 * Interface for defining control behaviors.
 * Implementations of this interface represent specific control actions
 * that can be executed based on gamepad input.
 */
public interface ControlDefinition {

    /**
     * Executes the control action based on the current gamepad state.
     *
     * @param gamepad The FTCLib GamepadEx object containing the current input state.
     */
    void run(GamepadEx gamepad);
}
