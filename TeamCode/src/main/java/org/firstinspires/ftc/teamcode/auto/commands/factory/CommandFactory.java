package org.firstinspires.ftc.teamcode.auto.commands.factory;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.auto.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;

public abstract class CommandFactory {
    public abstract Point getStartingPoint();

    public abstract SequentialCommandGroup getCommands();

    public abstract Class<? extends VLRSubsystem<?>>[] getRequiredSubsystems();
}
