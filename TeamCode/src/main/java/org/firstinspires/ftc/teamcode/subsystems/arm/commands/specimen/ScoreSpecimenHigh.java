package org.firstinspires.ftc.teamcode.subsystems.arm.commands.specimen;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmOverrideState;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmState;
import org.firstinspires.ftc.teamcode.helpers.commands.CustomConditionalCommand;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetCurrentArmState;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetRotatorAngle;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawAngle;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawState;

@Config
public class ScoreSpecimenHigh extends CustomConditionalCommand {
    public static double CLAW_ANGLE = 0.2;
    public static double ROTATOR = 55;
    public ScoreSpecimenHigh() {
        super(
                new SequentialCommandGroup(
                        new SetClawAngle(CLAW_ANGLE),
                        new WaitCommand(500),
                        new SetRotatorAngle(ROTATOR),
                        new WaitUntilCommand(VLRSubsystem.getInstance(ArmRotatorSubsystem.class)::reachedTargetPosition),
                        new SetClawState(ClawConfiguration.GripperState.OPEN),
                        new SetCurrentArmState(ArmState.State.SCORE_SPECIMEN_HIGH)
                ),
                () -> ArmState.isCurrentState(ArmState.State.PREPARE_SPECIMEN_HIGH) || ArmOverrideState.get()
        );
    }
}
