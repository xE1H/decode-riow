package org.firstinspires.ftc.teamcode.auto.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.arm.commands.RetractArm;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.sample.IntakeSample;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawAngle;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawState;

@Config
public class GrabBucketSample extends SequentialCommandGroup {
    public static double SLIDE = 0.2;
    public GrabBucketSample(){
        addCommands(
                new WaitCommand(100),
                new IntakeSample(SLIDE),
                new SetClawState(ClawConfiguration.GripperState.OPEN),
                new WaitCommand(400),
                new SetClawAngle(ClawConfiguration.VerticalRotation.DOWN),
                new WaitCommand(200),
                new SetClawState(ClawConfiguration.GripperState.CLOSED),
                new WaitCommand(300),
                new SetClawAngle(ClawConfiguration.VerticalRotation.UP),
                new WaitCommand(50),
                new RetractArm()
        );
    }
}
