package org.firstinspires.ftc.teamcode.subsystems.hang.commands;

import static org.firstinspires.ftc.teamcode.subsystems.arm.ArmState.State.HANG_SECOND_STAGE;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.helpers.commands.CustomConditionalCommand;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmState;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.RetractArm;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetArmOperationMode;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetCurrentArmState;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetRotatorAngle;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetSlideExtension;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawAngle;
import org.firstinspires.ftc.teamcode.subsystems.hang.HangConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.hang.HangSubsystem;

import java.util.function.BooleanSupplier;

public class ThirdStageHangCommand extends SequentialCommandGroup {
    public ThirdStageHangCommand(BooleanSupplier gamepadCondition) {
        addRequirements(VLRSubsystem.getRotator(), VLRSubsystem.getSlides());
        addCommands(
                new CustomConditionalCommand(
                        new RetractArm(),
                        () -> !ArmState.isCurrentState(ArmState.State.IN_ROBOT, ArmState.State.HANG_THIRD_STAGE, HANG_SECOND_STAGE)
                ),
                new SetCurrentArmState(HANG_SECOND_STAGE),
                new SetClawAngle(ClawConfiguration.VerticalRotation.UP),


                //new WaitCommand(500),
                //new ForceCalibrateSlides(),

                new SetRotatorAngle(101),
                new WaitUntilCommand(() -> VLRSubsystem.getRotator().getAngleDegrees() >= 60),
                new SetSlideExtension(0.314),
                new WaitUntilCommand(() -> (VLRSubsystem.getRotator().reachedTargetPosition() && VLRSubsystem.getSlides().reachedTargetPosition())).withTimeout(2000),


                new WaitUntilCommand(gamepadCondition),
                new SetArmOperationMode(ArmSlideConfiguration.OperationMode.HANG),
                new InstantCommand(() -> VLRSubsystem.getRotator().setHangCoefficients()),

                new SetSlideExtension(0.12),

                new WaitCommand(150),
                new SetRotatorAngle(92),
                new SetHangPosition(HangConfiguration.TargetPosition.UP),

                new WaitUntilCommand(() -> VLRSubsystem.getInstance(HangSubsystem.class).analogFeedbackThresholdReached()),

//                new ParallelRaceGroup(
//                        new WaitUntilCommand(()-> VLRSubsystem.getSlides().reachedTargetPosition()),
//                ),
//

                new SetSlideExtension(0.3),
                new SetArmOperationMode(ArmSlideConfiguration.OperationMode.NORMAL),
                new SetRotatorAngle(90),


                new WaitCommand(100000000),


                new InstantCommand(() -> VLRSubsystem.getRotator().setDefaultCoefficients()),
                new WaitCommand(100),

                new SetSlideExtension(0.29),
                new WaitCommand(300),

                new WaitCommand(1000),

                new SetSlideExtension(0.881),
                new WaitUntilCommand(() -> VLRSubsystem.getSlides().reachedTargetPosition()).withTimeout(1500),
                new InstantCommand(() -> VLRSubsystem.getRotator().setMappedCoefficients()),

                new SetRotatorAngle(98.2),
                new WaitUntilCommand(() -> VLRSubsystem.getRotator().reachedTargetPosition()).withTimeout(1000),


                new WaitUntilCommand(gamepadCondition),

                new SetSlideExtension(0.04),
                new WaitUntilCommand(() -> VLRSubsystem.getSlides().getExtension() < 0.78),
                new SetRotatorAngle(142),
                new WaitUntilCommand(() -> VLRSubsystem.getSlides().getExtension() < 0.38),
                new SetHangPosition(HangConfiguration.TargetPosition.DOWN),
                new SetRotatorAngle(38),
                new WaitUntilCommand(() -> VLRSubsystem.getSlides().reachedTargetPosition()).withTimeout(5000),


                new WaitUntilCommand(gamepadCondition),
                new SetRotatorAngle(130),
                new WaitCommand(100000)
        );
    }
}