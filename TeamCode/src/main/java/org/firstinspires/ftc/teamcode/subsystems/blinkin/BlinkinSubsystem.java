package org.firstinspires.ftc.teamcode.subsystems.blinkin;

import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.BLACK;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_PARTY_PALETTE;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;

//TODO: CHECKOUT COOL PATTERNS -> BEATS_PER_MINUTE_OCEAN_PALETTE, HEARTBEAT_WHITE

public class BlinkinSubsystem extends VLRSubsystem <BlinkinSubsystem>{
    private RevBlinkinLedDriver blinkinLedDriver;
    private RevBlinkinLedDriver.BlinkinPattern currentPattern;


    @Override
    public void initialize(HardwareMap hardwareMap){
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        currentPattern = COLOR_WAVES_PARTY_PALETTE;
        blinkinLedDriver.setPattern(currentPattern);
    }

    public void setPattern(RevBlinkinLedDriver.BlinkinPattern pattern){
        currentPattern = pattern;
        blinkinLedDriver.setPattern(pattern);

        logger.info("CURRENT PATTERN: " + currentPattern.toString());
    }

    public void next(){
        setPattern(currentPattern.next());
    }

    public void previous(){
        setPattern(currentPattern.previous());
    }

    public void disable(){
        setPattern(BLACK);
    }
}