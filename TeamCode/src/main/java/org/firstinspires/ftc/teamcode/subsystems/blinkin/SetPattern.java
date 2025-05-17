package org.firstinspires.ftc.teamcode.subsystems.blinkin;

import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_FOREST_PALETTE;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_OCEAN_PALETTE;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.BLUE;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_WHITE;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.RED;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.YELLOW;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.teamcode.helpers.commands.CustomConditionalCommand;
import org.firstinspires.ftc.teamcode.helpers.commands.LogCommand;
import org.firstinspires.ftc.teamcode.helpers.commands.RepeatNTimesCommand;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.limelight.LimelightYoloReader;

import java.util.logging.Level;

//TODO CONFIG MORE SIGMA PATTERNS
public class SetPattern extends InstantCommand {
    private final int waitBetweenBlinksMS = 60;

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

    public InstantCommand blinkSampleColour(RevBlinkinLedDriver.BlinkinPattern sampleColour){
        return new InstantCommand(()-> CommandScheduler.getInstance().schedule(
                new RepeatNTimesCommand(
                        3,
                        new SetPattern().blank(),
                        new WaitCommand(waitBetweenBlinksMS),
                        new SetPattern(sampleColour),
                        new WaitCommand(waitBetweenBlinksMS)
                )
        ));
    }

    public Command blinkSampleColour (LimelightYoloReader.Limelight.Sample.Color sampleColor){
        return new SequentialCommandGroup(
                new CustomConditionalCommand(
                        blinkSampleColour(RED),
                        ()-> sampleColor == LimelightYoloReader.Limelight.Sample.Color.RED
                ),
                new CustomConditionalCommand(
                        blinkSampleColour(BLUE),
                        ()-> sampleColor == LimelightYoloReader.Limelight.Sample.Color.BLUE
                ),
                new CustomConditionalCommand(
                        blinkSampleColour(YELLOW),
                        ()-> sampleColor == LimelightYoloReader.Limelight.Sample.Color.YELLOW
                ),
                new CustomConditionalCommand(
                        new LogCommand("SET PATTERN COMMAND", Level.SEVERE, "SOMETHING WHEN WRONG DISPLAYING LIMELIGHT SAMPLE COLOUR"),
                        ()-> (sampleColor != LimelightYoloReader.Limelight.Sample.Color.RED &&
                              sampleColor != LimelightYoloReader.Limelight.Sample.Color.BLUE &&
                              sampleColor != LimelightYoloReader.Limelight.Sample.Color.YELLOW)
                )
        );
    }
}