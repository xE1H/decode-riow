package org.firstinspires.ftc.teamcode.subsystems.arm.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmState;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration.TargetAngle;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration.TargetState;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawAngle;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawState;

public class MoveArmToIntake extends CustomConditionalCommand {

    public MoveArmToIntake(double extension) {
        super(
                new SequentialCommandGroup(
                        new CustomConditionalCommand(
                                new MoveArmInToRobot(),
                                () -> (ArmState.get() == ArmState.State.DEPOSIT || ArmState.get() == ArmState.State.SECOND_STAGE_HANG)
                        ),

                        new SetClawAngle(TargetAngle.UP),
                        new SetSlideExtension(extension),
                        new WaitUntilCommand(VLRSubsystem.getInstance(ArmRotatorSubsystem.class)::reachedTargetPosition),
                        new SetClawAngle(TargetAngle.DEPOSIT),
                        new SetClawState(TargetState.OPEN),
                        new SetArmState(ArmState.State.INTAKE)

                ),
                () -> ArmState.get() != ArmState.State.INTAKE

        );
        addRequirements(VLRSubsystem.getInstance(ArmRotatorSubsystem.class), VLRSubsystem.getInstance(ArmSlideSubsystem.class));

    }

    public MoveArmToIntake(){
        this(ArmSlideConfiguration.TargetPosition.INTAKE.extension);
    }

}