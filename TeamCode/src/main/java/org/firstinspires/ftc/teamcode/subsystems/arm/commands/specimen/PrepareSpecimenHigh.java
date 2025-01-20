package org.firstinspires.ftc.teamcode.subsystems.arm.commands.specimen;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import org.firstinspires.ftc.teamcode.helpers.commands.CustomConditionalCommand;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmState;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetCurrentArmState;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetRotatorAngle;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetSlideExtension;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawTwist;

@Config
public class PrepareSpecimenHigh extends CustomConditionalCommand {
    public static double ROTATOR = 60;
    public static double SLIDE =  0.3;
    public PrepareSpecimenHigh() {
        super(
                new SequentialCommandGroup(
                        new SetRotatorAngle(ROTATOR),
                        new SetClawTwist(ClawConfiguration.HorizontalRotation.NORMAL),
                        new SetSlideExtension(SLIDE),
                        new WaitUntilCommand(VLRSubsystem.getInstance(ArmSlideSubsystem.class)::reachedTargetPosition),
                        new SetCurrentArmState(ArmState.State.PREPARE_SPECIMEN_HIGH)
                ),
                () -> !ArmState.isCurrentState(ArmState.State.PREPARE_SPECIMEN_HIGH)
        );
    }
}

