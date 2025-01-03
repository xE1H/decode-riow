package org.firstinspires.ftc.teamcode.subsystems.hang.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.hang.HangConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.hang.HangSubsystem;

public class SetHangState extends InstantCommand {
    public SetHangState(HangConfiguration.TargetPosition target){
        super(()-> VLRSubsystem.getInstance(HangSubsystem.class).setTargetPosition(target));
    }
}
