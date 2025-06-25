package org.firstinspires.ftc.teamcode.helpers.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.command.WaitCommand;

import java.util.function.BooleanSupplier;

public class RepeatUntilCommand extends CommandBase {
    private final Command command;
    private final BooleanSupplier stopCondition;

    public RepeatUntilCommand(BooleanSupplier stopCondition, Command command) {
        this.command = command;
        this.stopCondition = stopCondition;

        // Declare the requirements of the inner command
        for (Subsystem subsystem : command.getRequirements()){
            addRequirements(subsystem);
        }
    }

    public RepeatUntilCommand(BooleanSupplier stopCondition, Command... commands) {
        this(stopCondition, new SequentialCommandGroup(commands));
    }

    @Override
    public void initialize() {
        command.initialize();
    }

    @Override
    public void execute() {
        if (command.isFinished()) {
            if (!stopCondition.getAsBoolean()) {
                command.end(false);
                command.initialize();
            }
        }
        else{
            command.execute();
        }
    }

    @Override
    public boolean isFinished() {
        return command.isFinished() && stopCondition.getAsBoolean();
    }

    @Override
    public void end(boolean interrupted) {
        command.end(interrupted);
    }
}