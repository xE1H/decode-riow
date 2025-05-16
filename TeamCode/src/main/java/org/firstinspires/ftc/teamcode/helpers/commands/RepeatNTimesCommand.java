package org.firstinspires.ftc.teamcode.helpers.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.Subsystem;

public class RepeatNTimesCommand extends CommandBase {
    private final Command command;
    private final int repeats;
    private int count = 0;

    public RepeatNTimesCommand(int repeats, Command command) {
        if (repeats <= 0)
            throw new IllegalArgumentException("Repeat count must be positive");

        this.command = command;
        this.repeats = repeats;

        for (Subsystem subsystem : command.getRequirements()) {
            addRequirements(subsystem);
        }
    }

    public RepeatNTimesCommand(int repeats, Command... commands) {
        this(repeats, new SequentialCommandGroup(commands));
    }

    @Override
    public void initialize() {
        count = 0;
        command.initialize();
    }

    @Override
    public void execute() {
        if (command.isFinished()) {
            count++;

            if (count < repeats) {
                command.end(false);
                command.initialize();
            }
        } else {
            command.execute();
        }
    }

    @Override
    public boolean isFinished() {
        return command.isFinished() && count >= repeats;
    }

    @Override
    public void end(boolean interrupted) {
        command.end(interrupted);
    }
}