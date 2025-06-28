package org.firstinspires.ftc.teamcode.subsystems.arm;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;

public class ResetRotator extends SequentialCommandGroup {
    public ResetRotator(){
        addCommands(
                new InstantCommand(()-> VLRSubsystem.getArm().enableRotatorPowerOverride(-0.3)),
                new WaitCommand(300),
                new InstantCommand(()-> VLRSubsystem.getArm().disableRotatorPowerOverride()),
                new InstantCommand(()-> VLRSubsystem.getArm().resetRotatorEncoder())
        );
    }
}
