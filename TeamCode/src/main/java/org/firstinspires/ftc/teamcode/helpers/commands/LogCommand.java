package org.firstinspires.ftc.teamcode.helpers.commands;

import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;

import java.util.function.Supplier;
import java.util.logging.Level;
import java.util.logging.Logger;

public class LogCommand extends InstantCommand {
    private final Logger logger;
    private final Level level;
    private final Supplier<String> messageSupplier;


    // Static message constructor (legacy)
    public LogCommand(String loggerName, Level level, String message) {
        this.logger = Logger.getLogger(loggerName);
        this.level = level;
        this.messageSupplier = () -> message;
    }

    public LogCommand(String loggerName, String message) {
        this(loggerName, Level.WARNING, message);
    }

    public LogCommand(Class<? extends VLRSubsystem<?>> subsystem, Level level, String message) {
        this(VLRSubsystem.getLogger(subsystem).getName(), level, message);
    }

    // NEW: Dynamic message constructor
    public LogCommand(String loggerName, Level level, Supplier<String> messageSupplier) {
        this.logger = Logger.getLogger(loggerName);
        this.level = level;
        this.messageSupplier = messageSupplier;
    }

    public LogCommand(Class<? extends VLRSubsystem<?>> subsystem, Level level, Supplier<String> messageSupplier) {
        this(VLRSubsystem.getLogger(subsystem).getName(), level, messageSupplier);
    }


    @Override
    public void run() {
        logger.log(level, messageSupplier.get());
    }
}
