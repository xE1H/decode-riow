package org.firstinspires.ftc.teamcode.subsystems.arm;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;

public class ResetSlides extends SequentialCommandGroup {
    public ResetSlides(){
        addCommands(
                new InstantCommand(()-> VLRSubsystem.getArm().enableSlidePowerOverride(-0.15)),
                new WaitCommand(750),
                new InstantCommand(()-> VLRSubsystem.getArm().disableSlidePowerOverride())
        );
    }
}
