package org.firstinspires.ftc.teamcode.subsystems.arm.commands.sample;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmOverrideState;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmState;
import org.firstinspires.ftc.teamcode.helpers.commands.CustomConditionalCommand;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.RetractArm;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetCurrentArmState;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetIsArmMoving;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetRotatorAngle;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetSlideExtension;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmLowState;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration.VerticalRotation;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawAngle;

@Config
public class ScoreSample extends CustomConditionalCommand {
    {
        // This needs to be here, since addRequirements needs to be called BEFORE the command is
        // able to run. The running may happen instantly (on super() being called), or at any point
        // in the future, so it's best to call addRequirements as soon as possible, in this case
        // before the constructor ever runs.
        addRequirements(VLRSubsystem.getInstance(ArmRotatorSubsystem.class), VLRSubsystem.getInstance(ArmSlideSubsystem.class));
    }

    public static int ROTATOR = 117;
    public static double SLIDE_HIGH = 1.02;
    public static double SLIDE_LOW = 0.4;

    public ScoreSample(int rotator) {
        super(new SequentialCommandGroup(
                        new SetIsArmMoving(),
                        new CustomConditionalCommand(
                                new RetractArm(),
                                () -> !ArmState.isCurrentState(ArmState.State.SAMPLE_SCORE, ArmState.State.IN_ROBOT)
                        ),

                        new SetRotatorAngle(rotator),
                        new WaitUntilCommand(() -> VLRSubsystem.getInstance(ArmRotatorSubsystem.class).getAngleDegrees() >= 30),
                        new SetClawAngle(VerticalRotation.DOWN),

                        new WaitUntilCommand(() -> VLRSubsystem.getInstance(ArmRotatorSubsystem.class).getAngleDegrees() >= rotator - 45),
                        new SetSlideExtension(ArmLowState.get() ? SLIDE_LOW : SLIDE_HIGH),
                        new WaitUntilCommand(VLRSubsystem.getInstance(ArmSlideSubsystem.class)::reachedTargetPosition),
                        new SetClawAngle(VerticalRotation.DEPOSIT),
                        new SetCurrentArmState(ArmState.State.SAMPLE_SCORE)
                ),
                () -> (ArmState.get() != ArmState.State.SAMPLE_SCORE && !ArmState.isMoving()) || ArmOverrideState.get()
        );
    }
}