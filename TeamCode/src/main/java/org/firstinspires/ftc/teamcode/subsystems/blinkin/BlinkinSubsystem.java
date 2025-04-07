package org.firstinspires.ftc.teamcode.subsystems.blinkin;

import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;

//TODO: CHECKOUT COOL PATTERNS -> BEATS_PER_MINUTE_OCEAN_PALETTE, HEARTBEAT_WHITE

public class BlinkinSubsystem extends VLRSubsystem <BlinkinSubsystem> {
    private RevBlinkinLedDriver blinkinLedDriver;
    private RevBlinkinLedDriver.BlinkinPattern currentPattern;


    @Override
    public void initialize(HardwareMap hardwareMap){
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        currentPattern = RAINBOW_RAINBOW_PALETTE;
        blinkinLedDriver.setPattern(currentPattern);
    }

    public void setPattern(RevBlinkinLedDriver.BlinkinPattern pattern){
        currentPattern = pattern;
        blinkinLedDriver.setPattern(pattern);
    }

    public void next(){
        currentPattern = currentPattern.next();
        blinkinLedDriver.setPattern(currentPattern);

        logger.info("CURRENT PATTERN: " + currentPattern.toString());
    }

    public void previous(){
        currentPattern = currentPattern.previous();
        blinkinLedDriver.setPattern(currentPattern);

        logger.info("CURRENT PATTERN: " + currentPattern.toString());
    }
}