package org.firstinspires.ftc.teamcode.helpers.commands;

import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;

import java.util.logging.Level;
import java.util.logging.Logger;

public class LogCommand extends InstantCommand {
    private final Logger logger;
    private final Level level;
    private final String message;

    public LogCommand(Class<? extends VLRSubsystem<?>> subsystem, Level level, String message) {
        this.logger = VLRSubsystem.getLogger(subsystem);
        this.level = level;
        this.message = message;
    }

    public LogCommand(String loggerName, Level level, String message) {
        this.logger = Logger.getLogger(loggerName);
        this.level = level;
        this.message = message;
    }

    @Override
    public void run() {
        logger.log(level, message);
    }
}
