package org.firstinspires.ftc.teamcode.subsystems.arm.commands.specimen;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.helpers.commands.CustomConditionalCommand;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmState;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.RetractArm;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetCurrentArmState;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetIsArmMoving;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetRotatorAngle;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetSlideExtension;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawAngle;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawState;

public class IntakeSpecimen extends CustomConditionalCommand {
    {
        // This needs to be here, since addRequirements needs to be called BEFORE the command is
        // able to run. The running may happen instantly (on super() being called), or at any point
        // in the future, so it's best to call addRequirements as soon as possible, in this case
        // before the constructor ever runs.
        addRequirements(VLRSubsystem.getInstance(ArmRotatorSubsystem.class), VLRSubsystem.getInstance(ArmSlideSubsystem.class));
    }

    public IntakeSpecimen() {
        super(new SequentialCommandGroup(
                        new CustomConditionalCommand(
                                new RetractArm(),
                                () -> !ArmState.isCurrentState(ArmState.State.INTAKE_SPECIMEN, ArmState.State.IN_ROBOT)
                        ),
                        new SetIsArmMoving(),
                        new SetRotatorAngle(ArmRotatorConfiguration.TargetAngle.INTAKE_SPECIMEN),
                        new WaitUntilCommand(()->VLRSubsystem.getInstance(ArmRotatorSubsystem.class).getAngleDegrees() > 15),
                        new SetSlideExtension(ArmSlideConfiguration.INTAKE_SPECIMEN),
                        new WaitCommand(1000),
                        new WaitUntilCommand(VLRSubsystem.getInstance(ArmSlideSubsystem.class)::reachedTargetPosition),
                        new SetCurrentArmState(ArmState.State.INTAKE_SPECIMEN)
                ),
                () -> !ArmState.isCurrentState(ArmState.State.INTAKE_SPECIMEN) && !ArmState.isMoving());
    }
}
