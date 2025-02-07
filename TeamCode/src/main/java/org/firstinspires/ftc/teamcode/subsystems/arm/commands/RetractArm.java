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
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration.HorizontalRotation;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration.VerticalRotation;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration.GripperState;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawAngle;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawState;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawTwist;
import org.firstinspires.ftc.teamcode.subsystems.neopixel.NeoPixelConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.neopixel.commands.SetColour;
import org.firstinspires.ftc.teamcode.subsystems.neopixel.commands.SetEffect;

public class RetractArm extends SequentialCommandGroup {
    public RetractArm() {
        if (ArmState.isMoving() && !ArmOverrideState.get()) return;
        ArmRotatorSubsystem arm = VLRSubsystem.getInstance(ArmRotatorSubsystem.class);
        ArmSlideSubsystem slides = VLRSubsystem.getInstance(ArmSlideSubsystem.class);
        addRequirements(arm, slides);

        if (ArmOverrideState.get()) {
            addCommands(
                    new SetClawState(GripperState.CLOSED),
                    new SetClawAngle(VerticalRotation.UP),
                    new WaitCommand(400),
                    new SetSlideExtension(ArmSlideConfiguration.TargetPosition.RETRACTED),
                    new WaitCommand(200),
                    new SetClawAngle(VerticalRotation.DEPOSIT),
                    new WaitUntilCommand(() -> slides.getPosition() < 280),
                    new SetClawAngle(VerticalRotation.UP),
                    new SetRotatorAngle(ArmRotatorConfiguration.TargetAngle.RETRACT),
                    new WaitUntilCommand(arm::reachedTargetPosition),
                    new SetCurrentArmState(
                            ArmState.State.IN_ROBOT
                    )
            );
            return;
        }

        addCommands(
                new SetIsArmMoving(),
                new CustomConditionalCommand(
                        new SequentialCommandGroup(
                                new SetEffect(NeoPixelConfiguration.Effect.CHASE_BACKWARD),
                                new SetColour(NeoPixelConfiguration.Colour.CYAN),

                                new SetClawState(GripperState.CLOSED),
                                new WaitCommand(90),
                                new SetClawAngle(VerticalRotation.UP),
                                new SetClawTwist(HorizontalRotation.NORMAL),
                                new WaitCommand(60),
                                new SetRotatorAngle(ArmRotatorConfiguration.TargetAngle.RETRACT), // Just in case the state gets bugged
                                new SetSlideExtension(ArmSlideConfiguration.TargetPosition.RETRACTED),
                                new WaitUntilCommand(slides::reachedTargetPosition),
                                new SetCurrentArmState(ArmState.State.IN_ROBOT)
                        ),
                        () -> ArmState.isCurrentState(ArmState.State.SAMPLE_INTAKE, ArmState.State.SPECIMEN_INTAKE)
                ),

                new CustomConditionalCommand(
                        new SequentialCommandGroup(
                                new SetEffect(NeoPixelConfiguration.Effect.CHASE_BACKWARD),
                                new SetColour(NeoPixelConfiguration.Colour.CYAN),

                                new SetClawState(GripperState.OPEN),
                                new WaitCommand(300),

                                new SetClawAngle(VerticalRotation.DOWN),
                                new WaitCommand(50),
                                new SetClawState(GripperState.CLOSED),

                                new SetSlideExtension(ArmSlideConfiguration.TargetPosition.RETRACTED),
                                new WaitCommand(200),
                                new SetClawAngle(VerticalRotation.DEPOSIT),
                                new WaitUntilCommand(slides::reachedTargetPosition),
                                new SetClawAngle(VerticalRotation.UP),
                                new SetRotatorAngle(ArmRotatorConfiguration.TargetAngle.RETRACT),
                                new WaitUntilCommand(arm::reachedTargetPosition),
                                new SetCurrentArmState(
                                        ArmState.State.IN_ROBOT
                                )
                        ),
                        () -> ArmState.isCurrentState(ArmState.State.SAMPLE_SCORE, ArmState.State.SPECIMEN_PREPARE)
                ),
                new CustomConditionalCommand(
                        new SequentialCommandGroup(
                                new SetClawState(GripperState.OPEN),
                                new WaitCommand(300),

                                new SetClawAngle(VerticalRotation.DOWN),
                                new WaitCommand(50),
                                new SetClawState(GripperState.CLOSED),
                                new SetClawAngle(VerticalRotation.DEPOSIT),
                                new SetSlideExtension(ArmSlideConfiguration.TargetPosition.RETRACTED),
                                new WaitCommand(200),
                                new SetClawAngle(VerticalRotation.DEPOSIT),
                                new WaitUntilCommand(slides::reachedTargetPosition),
                                new SetClawAngle(VerticalRotation.UP),
                                new SetRotatorAngle(ArmRotatorConfiguration.TargetAngle.RETRACT),
                                new WaitUntilCommand(arm::reachedTargetPosition),
                                new SetCurrentArmState(
                                        ArmState.State.IN_ROBOT
                                )
                        ),
                        () -> ArmState.isCurrentState(ArmState.State.SPECIMEN_SCORE)
                ),

                new CustomConditionalCommand(
                        new SequentialCommandGroup(
                                new SetClawAngle(VerticalRotation.UP),
                                new SetSlideExtension(ArmSlideConfiguration.TargetPosition.RETRACTED),
                                new WaitUntilCommand(slides::reachedTargetPosition),
                                new SetRotatorAngle(ArmRotatorConfiguration.TargetAngle.RETRACT),
                                new WaitUntilCommand(arm::reachedTargetPosition),
                                new SetCurrentArmState(ArmState.State.IN_ROBOT)
                        ),
                        () -> ArmState.get() == ArmState.State.HANG_SECOND_STAGE
                )
        );
    }
}
