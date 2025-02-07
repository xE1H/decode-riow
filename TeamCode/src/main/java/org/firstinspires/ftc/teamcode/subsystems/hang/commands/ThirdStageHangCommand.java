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
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetRotatorAngle;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetSlideExtension;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawAngle;
import org.firstinspires.ftc.teamcode.subsystems.hang.HangConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.hang.HangSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.neopixel.NeoPixelConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.neopixel.commands.SetColour;
import org.firstinspires.ftc.teamcode.subsystems.neopixel.commands.SetEffect;

import java.util.function.BooleanSupplier;

public class ThirdStageHangCommand extends SequentialCommandGroup {
    public ThirdStageHangCommand(BooleanSupplier gamepadCondition) {
        addRequirements(VLRSubsystem.getRotator(), VLRSubsystem.getSlides(), VLRSubsystem.getHang());
        addCommands(
                new CustomConditionalCommand(
                        new RetractArm(),
                        () -> !ArmState.isCurrentState(ArmState.State.IN_ROBOT, ArmState.State.HANG_THIRD_STAGE, HANG_SECOND_STAGE)
                ),
                //new SetCurrentArmState(SECOND_STAGE_HANG),
                new SetClawAngle(ClawConfiguration.VerticalRotation.UP),


                //new ForceCalibrateSlides(),

                new SetRotatorAngle(102.5),
                new WaitUntilCommand(()-> VLRSubsystem.getRotator().getAngleDegrees() >= 60),
                new SetSlideExtension(0.314),
                new WaitUntilCommand(()-> (VLRSubsystem.getRotator().reachedTargetPosition() && VLRSubsystem.getSlides().reachedTargetPosition())).withTimeout(2000),

                //LEDS:
                new SetColour(NeoPixelConfiguration.Colour.RED),
                new SetEffect(NeoPixelConfiguration.Effect.SOLID_COLOR),


                new WaitUntilCommand(gamepadCondition),
                new SetSlideExtension(0.27),
                new WaitCommand(300),

                new SetArmOperationMode(ArmSlideConfiguration.OperationMode.HANG_SLOW),
                new SetSlideExtension(0.2),
                new WaitCommand(100),

                new SetRotatorAngle(89.75),
                new SetHangPosition(HangConfiguration.TargetPosition.UP),
                new WaitCommand(50),

                new WaitUntilCommand(()-> VLRSubsystem.getInstance(HangSubsystem.class).analogFeedbackThresholdReached()),

                //LEDS:
                new SetColour(NeoPixelConfiguration.Colour.GREEN),


                new InstantCommand(()-> VLRSubsystem.getInstance(HangSubsystem.class).setPower(0)),
                new SetArmOperationMode(ArmSlideConfiguration.OperationMode.NORMAL),

                new SetSlideExtension(0.3),
                new WaitCommand(100),

                new SetSlideExtension(0.888),
                new WaitCommand(150),
                new SetRotatorAngle(85),

                new WaitUntilCommand(()-> (VLRSubsystem.getRotator().reachedTargetPosition() && VLRSubsystem.getSlides().reachedTargetPosition())).withTimeout(2000),
                new InstantCommand(()-> VLRSubsystem.getRotator().setMappedCoefficients()),

                new SetRotatorAngle(102),
                new WaitCommand(360),

                new SetSlideExtension(0.83),
                new WaitCommand(300),
                new SetRotatorAngle(99),


                new WaitUntilCommand(gamepadCondition),
                new SetEffect(NeoPixelConfiguration.Effect.CHASE_FORWARD),
                new SetColour(NeoPixelConfiguration.Colour.PURPLE),



                new SetArmOperationMode(ArmSlideConfiguration.OperationMode.HANG_FAST),

                new SetSlideExtension(0.04),
                new WaitUntilCommand(() -> VLRSubsystem.getSlides().getExtension() < 0.79),
                new SetRotatorAngle(133),
                new WaitUntilCommand(() -> VLRSubsystem.getSlides().getExtension() < 0.28),
                new SetHangPosition(HangConfiguration.TargetPosition.DOWN),

                new SetRotatorAngle(35),
                new WaitUntilCommand(() -> VLRSubsystem.getSlides().reachedTargetPosition()).withTimeout(3000),


                new SetEffect(NeoPixelConfiguration.Effect.CHASE_BACKWARD),
                new SetColour(NeoPixelConfiguration.Colour.GREEN),


                new WaitUntilCommand(gamepadCondition),
                new SetRotatorAngle(120)
        );
    }
}