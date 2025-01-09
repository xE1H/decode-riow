package org.firstinspires.ftc.teamcode.helpers.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.arm.commands.MoveArmInToRobot;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.MoveArmToIntake;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawAngle;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawState;

public class GrabBucketSample extends SequentialCommandGroup {
    public GrabBucketSample(){
        addCommands(
                new WaitCommand(100),
                new MoveArmToIntake(0.62),
                new SetClawState(ClawConfiguration.TargetState.OPEN),
                new WaitCommand(600),
                new SetClawAngle(ClawConfiguration.TargetAngle.DOWN),
                new WaitCommand(200),
                new SetClawState(ClawConfiguration.TargetState.CLOSED),
                new WaitCommand(100),
                new SetClawAngle(ClawConfiguration.TargetAngle.UP),
                new WaitCommand(50),
                new MoveArmInToRobot()
        );
    }
}
