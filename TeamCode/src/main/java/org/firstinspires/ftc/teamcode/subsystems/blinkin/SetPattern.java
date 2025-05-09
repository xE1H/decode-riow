package org.firstinspires.ftc.teamcode.subsystems.blinkin;

import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_FOREST_PALETTE;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_OCEAN_PALETTE;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_WHITE;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;

//TODO CONFIG MORE SIGMA PATTERNS
public class SetPattern extends InstantCommand {
    public SetPattern(RevBlinkinLedDriver.BlinkinPattern pattern){
        super(()-> VLRSubsystem.getInstance(BlinkinSubsystem.class).setPattern(pattern));
    }

    public SetPattern(){}

    public Command red(){
        return new SetPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_RED);
    }

    public Command green(){
        return new SetPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
    }

    public Command blank(){
        return new SetPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
    }

    public Command oceanPalette(){
        return new SetPattern(BEATS_PER_MINUTE_OCEAN_PALETTE);
    }

    public Command forestPalette() {return new SetPattern(BEATS_PER_MINUTE_FOREST_PALETTE);}

    public Command heartbeatWhite(){
        return new SetPattern(HEARTBEAT_WHITE);
    }

    public Command rainbow(){
        return new SetPattern(RAINBOW_RAINBOW_PALETTE);
    }
}
