package org.firstinspires.ftc.teamcode.helpers.commands;

import com.arcrobotics.ftclib.command.CommandBase;

/**
 * An abstract command that executes a single action and then finishes immediately.
 * This is useful for commands that need to perform a quick, one-time action.
 */
public abstract class InstantCommand extends CommandBase {
    /**
     * The action to be performed by this command.
     * This method should be implemented by subclasses to define the specific action.
     */
    public abstract void run();

    /**
     * Initializes the command by executing the run method.
     * This method is called once when the command is scheduled.
     */
    @Override
    public void initialize() {
        run();
    }

    /**
     * Indicates whether the command has finished.
     * For InstantCommand, this should always return true as the command finishes immediately after execution.
     *
     * @return true, indicating the command has finished.
     */
    @Override
    public boolean isFinished() {
        return true;
    }
}