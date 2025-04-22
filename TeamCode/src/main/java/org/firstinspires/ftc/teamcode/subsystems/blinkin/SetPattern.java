package org.firstinspires.ftc.teamcode.subsystems.blinkin;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;

//TODO CONFIG MORE SIGMA PATTERNS
public class SetPattern extends InstantCommand {

    public SetPattern(RevBlinkinLedDriver.BlinkinPattern pattern){
        super(()-> VLRSubsystem.getInstance(BlinkinSubsystem.class).setPattern(pattern));
    }

    public Command red(){
        return new SetPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
    }

    public Command blank(){
        return new SetPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
    }
}
