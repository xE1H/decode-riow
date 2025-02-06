package org.firstinspires.ftc.teamcode.subsystems.arm.commands.specimen;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.helpers.commands.InstantCommand;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmState;
import org.firstinspires.ftc.teamcode.helpers.commands.CustomConditionalCommand;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetCurrentArmState;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetRotatorAngle;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawAngle;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawState;

@Config
public class ScoreSpecimenHigh extends CustomConditionalCommand {
    {
        // This needs to be here, since addRequirements needs to be called BEFORE the command is
        // able to run. The running may happen instantly (on super() being called), or at any point
        // in the future, so it's best to call addRequirements as soon as possible, in this case
        // before the constructor ever runs.
        addRequirements(VLRSubsystem.getInstance(ArmRotatorSubsystem.class), VLRSubsystem.getInstance(ArmSlideSubsystem.class));
    }

    public static double CLAW_ANGLE = 0.8;
    public static double ROTATOR_ANGLE = 45;
    public static int ROTATOR_TIMEOUT = 400;

    public ScoreSpecimenHigh() {
        super(new SequentialCommandGroup(
                        new SetClawAngle(CLAW_ANGLE),
                        new WaitCommand(200),
                        new InstantCommand() {
                            @Override
                            public void run() {
                                VLRSubsystem.getInstance(ArmRotatorSubsystem.class).setHangCoefficients();
                            }
                        },
                        new SetRotatorAngle(ROTATOR_ANGLE),
                        new ParallelRaceGroup(
                                new WaitUntilCommand(VLRSubsystem.getInstance(ArmRotatorSubsystem.class)::reachedTargetPosition),
                                new WaitCommand(ROTATOR_TIMEOUT)
                        ),
                        new InstantCommand() {
                            @Override
                            public void run() {
                                VLRSubsystem.getInstance(ArmRotatorSubsystem.class).setDefaultCoefficients();
                            }
                        },
                        new WaitCommand(400),
                        new SetClawState(ClawConfiguration.GripperState.OPEN),
                        new SetCurrentArmState(ArmState.State.SPECIMEN_SCORE)
                ),
                () -> (!ArmState.isCurrentState(ArmState.State.SPECIMEN_SCORE)));
    }
}