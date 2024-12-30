package org.firstinspires.ftc.teamcode.subsystems.arm.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmState;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration.TargetAngle;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawAngle;

public class MoveArmToDeposit extends CustomConditionalCommand {

    public MoveArmToDeposit() {
        super(
                new SequentialCommandGroup(
                        new CustomConditionalCommand(
                                new MoveArmInToRobot(),
                                () -> ArmState.get() == ArmState.State.INTAKE
                        ),

                        new SetRotatorAngle(ArmRotatorConfiguration.TargetAngle.DEPOSIT),
                        new WaitUntilCommand(() -> VLRSubsystem.getInstance(ArmRotatorSubsystem.class).getAngleDegrees() >= 30),
                        new SetClawAngle(TargetAngle.DOWN),

                        new WaitUntilCommand(VLRSubsystem.getInstance(ArmRotatorSubsystem.class)::reachedTargetPosition),
                        new SetSlideExtension(ArmSlideConfiguration.TargetPosition.DEPOSIT),

                        new WaitUntilCommand(VLRSubsystem.getInstance(ArmSlideSubsystem.class)::reachedTargetPosition),
                        new SetClawAngle(TargetAngle.DEPOSIT),
                        new SetArmState(ArmState.State.DEPOSIT)
                ),
                ()->ArmState.get() != ArmState.State.DEPOSIT
        );
        addRequirements(VLRSubsystem.getInstance(ArmRotatorSubsystem.class), VLRSubsystem.getInstance(ArmSlideSubsystem.class));
    }
}