package org.firstinspires.ftc.teamcode.subsystems.arm.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmState;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration.TargetTwist;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration.TargetAngle;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration.TargetState;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawAngle;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawState;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawTwist;

public class MoveArmInToRobot extends SequentialCommandGroup {
    public MoveArmInToRobot() {
            ArmRotatorSubsystem arm = VLRSubsystem.getInstance(ArmRotatorSubsystem.class);
            ArmSlideSubsystem slides = VLRSubsystem.getInstance(ArmSlideSubsystem.class);
            addRequirements(arm, slides);


            addCommands(
                    new CustomConditionalCommand(
                            new SequentialCommandGroup(
                                    new SetClawState(TargetState.CLOSED),
                                    new WaitCommand(100),
                                    new SetClawAngle(TargetAngle.UP),
                                    new SetClawTwist(TargetTwist.NORMAL),
                                    new WaitCommand(80),
                                    new SetSlideExtension(ArmSlideConfiguration.TargetPosition.RETRACTED),
                                    new WaitUntilCommand(slides::reachedTargetPosition),
                                    new SetArmState(ArmState.State.IN_ROBOT)
                            ),
                            () -> ArmState.get() == ArmState.State.INTAKE
                    ),

                    new CustomConditionalCommand(
                            new SequentialCommandGroup(
                                    new SetClawState(TargetState.OPEN),
                                    new WaitCommand(200),
                                    new SetClawAngle(TargetAngle.DOWN),
                                    new WaitCommand(100),
                                    new SetClawState(TargetState.CLOSED),
                                    new SetSlideExtension(ArmSlideConfiguration.TargetPosition.RETRACTED),
                                    new WaitCommand(200),
                                    new SetClawAngle(TargetAngle.DEPOSIT),
                                    new WaitUntilCommand(slides::reachedTargetPosition),
                                    new SetClawAngle(TargetAngle.UP),
                                    new SetRotatorAngle(ArmRotatorConfiguration.TargetAngle.DOWN),
                                    new WaitUntilCommand(arm::reachedTargetPosition),
                                    new SetArmState(
                                            ArmState.State.IN_ROBOT
                                    )
                            ),
                            () -> ArmState.get() == ArmState.State.DEPOSIT
                    ),

                    new CustomConditionalCommand(
                            new SequentialCommandGroup(
                                    new SetClawAngle(TargetAngle.UP),
                                    new SetSlideExtension(ArmSlideConfiguration.TargetPosition.RETRACTED),
                                    new WaitUntilCommand(slides::reachedTargetPosition),
                                    new SetRotatorAngle(ArmRotatorConfiguration.TargetAngle.DOWN),
                                    new WaitUntilCommand(arm::reachedTargetPosition),
                                    new SetArmState(ArmState.State.IN_ROBOT)
                            ),
                            () -> ArmState.get() == ArmState.State.SECOND_STAGE_HANG
                    )
            );
    }
}
