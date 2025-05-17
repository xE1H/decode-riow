package org.firstinspires.ftc.teamcode.subsystems.hang.commands;//package org.firstinspires.ftc.teamcode.subsystems.hang.commands;
//
//import static org.firstinspires.ftc.teamcode.subsystems.arm.ArmState.State.HANG_SECOND_STAGE;
//
//import com.arcrobotics.ftclib.command.InstantCommand;
//import com.arcrobotics.ftclib.command.SequentialCommandGroup;
//import com.arcrobotics.ftclib.command.WaitCommand;
//import com.arcrobotics.ftclib.command.WaitUntilCommand;
//
//import org.firstinspires.ftc.teamcode.helpers.commands.CustomConditionalCommand;
//import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.arm.ArmState;
//import org.firstinspires.ftc.teamcode.subsystems.arm.commands.RetractArm;
//import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetArmOperationMode;
//import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetRotatorAngle;
//import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetSlideExtension;
//import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration;
//import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
//import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawAngle;
//
//import java.util.function.BooleanSupplier;
//
//public class SecondStageHangCommand extends SequentialCommandGroup {
//    public SecondStageHangCommand(BooleanSupplier gamepadCondition) {
//        addRequirements(VLRSubsystem.getRotator(), VLRSubsystem.getSlides(), VLRSubsystem.getHang());
//        addCommands(
//                new CustomConditionalCommand(
//                        new RetractArm(),
//                        () -> !ArmState.isCurrentState(ArmState.State.IN_ROBOT, ArmState.State.HANG_THIRD_STAGE, HANG_SECOND_STAGE)
//                ),
//                //new SetCurrentArmState(SECOND_STAGE_HANG),
//                new SetClawAngle(ClawConfiguration.VerticalRotation.UP),
//
//
//                //new ForceCalibrateSlides(),
//
//                new SetRotatorAngle(102.5),
//                new WaitUntilCommand(() -> VLRSubsystem.getRotator().getAngleDegrees() >= 60),
//                new SetSlideExtension(0.314),
//                new WaitUntilCommand(() -> (VLRSubsystem.getRotator().reachedTargetPosition() && VLRSubsystem.getSlides().reachedTargetPosition())).withTimeout(2000),
//
//                //LEDS:
//                //new SetColour(NeoPixelConfiguration.Colour.RED),
//                //new SetEffect(NeoPixelConfiguration.Effect.SOLID_COLOR),
//
//                new WaitUntilCommand(gamepadCondition),
//                new SetSlideExtension(0.1),
//                new InstantCommand(() -> VLRSubsystem.getRotator().setHangCoefficients()),
//                new SetRotatorAngle(42),
//                new WaitCommand(1000),
//                new SetArmOperationMode(ArmSlideConfiguration.OperationMode.HANG_SLOW),
//                new SetSlideExtension(0.03)
//        );
//    }
//}