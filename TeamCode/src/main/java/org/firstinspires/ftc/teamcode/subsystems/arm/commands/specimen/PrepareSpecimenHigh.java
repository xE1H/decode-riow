package org.firstinspires.ftc.teamcode.subsystems.arm.commands.specimen;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import org.firstinspires.ftc.teamcode.helpers.commands.CustomConditionalCommand;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmState;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetCurrentArmState;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetRotatorAngle;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetSlideExtension;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawTwist;

public class PrepareSpecimenHigh extends CustomConditionalCommand {
    public PrepareSpecimenHigh() {
        super(
                new SequentialCommandGroup(
                        new SetRotatorAngle(ArmRotatorConfiguration.TargetAngle.PREPARE_SPECIMEN_HIGH),
                        new WaitUntilCommand(() -> VLRSubsystem.getInstance(ArmRotatorSubsystem.class).getAngleDegrees() >= 60),
                        new SetClawTwist(ClawConfiguration.TargetTwist.NORMAL),
                        new SetSlideExtension(ArmSlideConfiguration.TargetPosition.PREPARE_SPECIMEN_HIGH),
                        new WaitUntilCommand(VLRSubsystem.getInstance(ArmSlideSubsystem.class)::reachedTargetPosition),
                        new SetCurrentArmState(ArmState.State.PREPARE_SPECIMEN_HIGH)
                ),
                () -> !ArmState.isCurrentState(ArmState.State.PREPARE_SPECIMEN_HIGH)
        );
    }
}

