package org.firstinspires.ftc.teamcode.auto.commands;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.RetractArm;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.sample.ScoreSample;

public class ScoreHighBucketSample extends SequentialCommandGroup {
    public ScoreHighBucketSample(){
        addCommands(
                new ScoreSample(117),
                new WaitCommand(80),
                new RetractArm()
        );
    }
}
