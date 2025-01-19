package org.firstinspires.ftc.teamcode.subsystems.arm.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.helpers.commands.CustomConditionalCommand;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmOverrideState;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmState;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration.TargetTwist;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration.TargetAngle;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration.TargetState;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawAngle;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawState;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawTwist;

public class RetractArm extends SequentialCommandGroup {
    public RetractArm() {
        if (ArmState.isMoving() && !ArmOverrideState.get()) return;
        ArmRotatorSubsystem arm = VLRSubsystem.getInstance(ArmRotatorSubsystem.class);
        ArmSlideSubsystem slides = VLRSubsystem.getInstance(ArmSlideSubsystem.class);
        addRequirements(arm, slides);
        addCommands(
                new SetIsArmMoving(),
                new CustomConditionalCommand(
                        new SequentialCommandGroup(
                                new SetClawTwist(TargetTwist.NORMAL),
                                new SetClawState(TargetState.CLOSED),
                                new WaitCommand(100),
                                new SetClawAngle(TargetAngle.UP),
                                new WaitCommand(80),
                                new SetRotatorAngle(ArmRotatorConfiguration.TargetAngle.RETRACT), // Just in case the state gets bugged
                                new SetSlideExtension(ArmSlideConfiguration.TargetPosition.RETRACTED),
                                new WaitUntilCommand(slides::reachedTargetPosition),
                                new SetCurrentArmState(ArmState.State.IN_ROBOT)
                        ),
                        () -> ArmState.isCurrentState(ArmState.State.INTAKE_SAMPLE, ArmState.State.INTAKE_SPECIMEN)
                ),

                new CustomConditionalCommand(
                        new SequentialCommandGroup(
                                new SetClawState(TargetState.OPEN),
                                new WaitCommand(200),
                                new SetClawState(TargetState.CLOSED),
                                new WaitCommand(100),
                                new SetClawAngle(TargetAngle.DOWN),
                                new SetSlideExtension(ArmSlideConfiguration.TargetPosition.RETRACTED),
                                new WaitCommand(200),
                                new SetClawAngle(TargetAngle.DEPOSIT),
                                new WaitUntilCommand(slides::reachedTargetPosition),
                                new SetClawAngle(TargetAngle.UP),
                                new SetRotatorAngle(ArmRotatorConfiguration.TargetAngle.RETRACT),
                                new WaitUntilCommand(arm::reachedTargetPosition),
                                new SetCurrentArmState(
                                        ArmState.State.IN_ROBOT
                                )
                        ),
                        () -> ArmState.isCurrentState(ArmState.State.SCORE_SAMPLE_LOW, ArmState.State.SCORE_SAMPLE_HIGH, ArmState.State.PREPARE_SPECIMEN_HIGH, ArmState.State.SCORE_SPECIMEN_HIGH, ArmState.State.PREPARE_SPECIMEN_LOW, ArmState.State.SCORE_SPECIMEN_LOW)
                ),

                new CustomConditionalCommand(
                        new SequentialCommandGroup(
                                new SetClawAngle(TargetAngle.UP),
                                new SetSlideExtension(ArmSlideConfiguration.TargetPosition.RETRACTED),
                                new WaitUntilCommand(slides::reachedTargetPosition),
                                new SetRotatorAngle(ArmRotatorConfiguration.TargetAngle.RETRACT),
                                new WaitUntilCommand(arm::reachedTargetPosition),
                                new SetCurrentArmState(ArmState.State.IN_ROBOT)
                        ),
                        () -> ArmState.get() == ArmState.State.SECOND_STAGE_HANG
                )
        );
    }
}
