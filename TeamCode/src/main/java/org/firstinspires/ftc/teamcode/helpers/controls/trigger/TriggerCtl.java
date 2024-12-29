package org.firstinspires.ftc.teamcode.helpers.controls.trigger;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.helpers.controls.ControlDefinition;

import java.util.function.Consumer;

/**
 * Implements a control definition for trigger inputs.
 * This class allows mapping of trigger inputs to specific actions.
 */
public class TriggerCtl implements ControlDefinition {
    /**
     * The trigger being monitored.
     */
    GamepadKeys.Trigger trigger;

    /**
     * The action to be performed when the trigger is activated.
     */
    Consumer<Double> action;

    /**
     * Constructs a new TriggerCtl instance.
     *
     * @param trigger The trigger to monitor (LEFT_TRIGGER or RIGHT_TRIGGER).
     * @param action  The action to perform when the trigger is activated.
     *                The action accepts a Double parameter representing the trigger's position (0.0 to 1.0).
     */
    public TriggerCtl(GamepadKeys.Trigger trigger, Consumer<Double> action) {
        this.trigger = trigger;
        this.action = action;
    }

    /**
     * Executes the defined action based on the current trigger state.
     *
     * @param gamepad The FTCLib GamepadEx object containing the current input state.
     */
    @Override
    public void run(GamepadEx gamepad) {
        action.accept(gamepad.getTrigger(trigger));
    }
}
