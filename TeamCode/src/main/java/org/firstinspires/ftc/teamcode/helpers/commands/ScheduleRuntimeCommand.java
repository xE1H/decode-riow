package org.firstinspires.ftc.teamcode.helpers.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Subsystem;

import java.util.Set;
import java.util.function.Supplier;

/**
 * A wrapper command that creates command on runtime, not on scheduling
 * <p>
 * Useful for commands that use robot's current states, angles or other data and we want the command scheduler
 * to check them on runtime, not on scheduling
 */

public class ScheduleRuntimeCommand extends CommandBase {
    private Command actualCommand;
    Supplier<Command> commandSupplier;

    public ScheduleRuntimeCommand (Supplier<Command> commandSupplier) {
        this.commandSupplier = commandSupplier;
    }

    @Override
    public void initialize() {
        actualCommand = commandSupplier.get();
        if (actualCommand != null) {actualCommand.initialize();}
    }

    @Override
    public void execute() {
        if (actualCommand != null) {actualCommand.execute();}
    }

    @Override
    public boolean isFinished() {return actualCommand == null || actualCommand.isFinished();}

    @Override
    public void end(boolean interrupted) {if (actualCommand != null) {actualCommand.end(interrupted);}}

    @Override
    public Set<Subsystem> getRequirements() {return commandSupplier.get().getRequirements();}

}
