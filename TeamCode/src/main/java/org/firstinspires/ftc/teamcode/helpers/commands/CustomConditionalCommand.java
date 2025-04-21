package org.firstinspires.ftc.teamcode.helpers.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;

import java.util.function.BooleanSupplier;

public class CustomConditionalCommand extends ConditionalCommand {
    /**
     * Creates a new ConditionalCommand.
     *
     * @param onTrue    the command to run if the condition is true
     * @param onFalse   the command to run if the condition is false
     * @param condition the condition to determine which command to run
     */
    public CustomConditionalCommand(Command onTrue, Command onFalse, BooleanSupplier condition) {
        super(onTrue, onFalse, condition);
    }

    public CustomConditionalCommand(Command onTrue, BooleanSupplier condition) {
        super(onTrue, new InstantCommand(), condition);
    }

    public CustomConditionalCommand(BooleanSupplier condition, Command onTrue) {
        super(onTrue, new InstantCommand(), condition);
    }
}
