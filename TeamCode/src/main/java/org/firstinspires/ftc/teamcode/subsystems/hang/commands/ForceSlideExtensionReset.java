package org.firstinspires.ftc.teamcode.subsystems.hang.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;

public class ForceSlideExtensionReset extends SequentialCommandGroup {
    public ForceSlideExtensionReset(){
        addRequirements(VLRSubsystem.getSlides());
        addCommands(
                new InstantCommand(()-> VLRSubsystem.getSlides().setUnreachablePosForCalibration()),
                new WaitCommand(1000)
        );
    }
}
