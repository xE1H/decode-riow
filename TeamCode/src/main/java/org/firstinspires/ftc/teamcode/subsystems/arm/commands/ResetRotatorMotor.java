package org.firstinspires.ftc.teamcode.subsystems.arm.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.helpers.commands.InstantCommand;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorSubsystem;

public class ResetRotatorMotor extends SequentialCommandGroup {
    public ResetRotatorMotor() {
        ArmRotatorSubsystem ars = VLRSubsystem.getInstance(ArmRotatorSubsystem.class);
        addCommands(
                new InstantCommand() {
                    @Override
                    public void run() {
                        ars.disableMotor();
                    }
                },
                new WaitCommand(1000),
                new InstantCommand() {
                    @Override
                    public void run() {
                        ars.reenableMotor();
                    }
                }
        );
    }
}
