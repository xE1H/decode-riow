package org.firstinspires.ftc.teamcode.subsystems.arm.commands.specimen;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmState;
import org.firstinspires.ftc.teamcode.helpers.commands.CustomConditionalCommand;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetCurrentArmState;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetSlideExtension;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideSubsystem;

public class ScoreSpecimenHigh extends CustomConditionalCommand {
    public ScoreSpecimenHigh() {
        super(
                new SequentialCommandGroup(
                        new SetSlideExtension(ArmSlideConfiguration.TargetPosition.SCORE_SPECIMEN_HIGH),
                        new WaitUntilCommand(VLRSubsystem.getInstance(ArmSlideSubsystem.class)::reachedTargetPosition),
                        new SetCurrentArmState(ArmState.State.SCORE_SPECIMEN_HIGH)
                ),
                () -> !ArmState.isCurrentState(ArmState.State.SCORE_SPECIMEN_HIGH)
        );
    }
}
