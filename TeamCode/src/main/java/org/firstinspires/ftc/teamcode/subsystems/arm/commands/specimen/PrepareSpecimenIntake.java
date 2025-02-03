package org.firstinspires.ftc.teamcode.subsystems.arm.commands.specimen;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.helpers.commands.CustomConditionalCommand;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmState;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetCurrentArmState;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetRotatorAngle;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetSlideExtension;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawAngle;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawState;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawTwist;

@Config
public class PrepareSpecimenIntake extends SequentialCommandGroup {
    {
        // This needs to be here, since addRequirements needs to be called BEFORE the command is
        // able to run. The running may happen instantly (on super() being called), or at any point
        // in the future, so it's best to call addRequirements as soon as possible, in this case
        // before the constructor ever runs.
        addRequirements(VLRSubsystem.getInstance(ArmRotatorSubsystem.class), VLRSubsystem.getInstance(ArmSlideSubsystem.class));
    }

    public static double ROTATOR = 19.3;
    public static double SLIDE = 0.265;
    public static double CLAW_ANGLE = 0.57;

    public PrepareSpecimenIntake(){
        addCommands(
                new CustomConditionalCommand(
                        new ParallelCommandGroup(
                                new SetRotatorAngle(ROTATOR),
                                new SetSlideExtension(SLIDE),
                                new SetClawAngle(CLAW_ANGLE),
                                new SetClawState(ClawConfiguration.GripperState.OPEN),
                                new SetCurrentArmState(ArmState.State.SPECIMEN_INTAKE)
                        ),
                        () -> !ArmState.isCurrentState(ArmState.State.SPECIMEN_INTAKE)
                )
        );
    }
}
