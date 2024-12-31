package org.firstinspires.ftc.teamcode.subsystems.hang;

import com.arcrobotics.ftclib.command.InstantCommand;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;

public class SetHangState extends InstantCommand {
    public SetHangState(HangConfiguration.TargetPosition target){
        super(()-> VLRSubsystem.getInstance(HangSubsystem.class).setTargetPosition(target));
    }
}
