package org.firstinspires.ftc.teamcode.subsystems.arm.commands.specimen;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.auto.pedroCommands.FollowPath;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.helpers.commands.CustomConditionalCommand;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmState;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetCurrentArmState;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetRotatorAngle;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetSlideExtension;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawAngle;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawTwist;

@Config
public class PrepareSpecimenHigh extends ParallelCommandGroup {
    public static double ROTATOR = 60;
    public static double SLIDE = 0.3;
    public static double CLAW_ANGLE = 0.4;

    public PrepareSpecimenHigh() {
        addCommands(
                new CustomConditionalCommand(
                        new ParallelCommandGroup(
                                new SetClawAngle(CLAW_ANGLE),
                                new SetRotatorAngle(ROTATOR),
                                new SetClawTwist(ClawConfiguration.HorizontalRotation.NORMAL),
                                new SequentialCommandGroup(
                                        new SetSlideExtension(SLIDE),
                                        new WaitUntilCommand(VLRSubsystem.getInstance(ArmSlideSubsystem.class)::reachedTargetPosition)),
                                new SetCurrentArmState(ArmState.State.SPECIMEN_PREPARE)
                        ),
                        () -> !ArmState.isCurrentState(ArmState.State.SPECIMEN_PREPARE)
                )
        );

    }

}

