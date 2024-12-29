package org.firstinspires.ftc.teamcode.subsystems.arm.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration.ArmState;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration.TargetTwist;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration.TargetAngle;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration.TargetState;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawAngle;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawState;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawTwist;

public class SetArmState_InRobot extends SequentialCommandGroup {

    public SetArmState_InRobot() {
        ArmRotatorSubsystem arm = VLRSubsystem.getInstance(ArmRotatorSubsystem.class);
        ArmSlideSubsystem slides = VLRSubsystem.getInstance(ArmSlideSubsystem.class);
        ArmState currentState = arm.getArmState();

        if (currentState == ArmState.INTAKE){
            addCommands(
                    new SetClawState(TargetState.CLOSED_NORMAL),
                    new WaitCommand(100),
                    new SetClawAngle(TargetAngle.UP),
                    new SetClawTwist(TargetTwist.NORMAL),
                    new WaitCommand(80),
                    new SetSlideExtension(ArmSlideConfiguration.TargetPosition.RETRACTED),
                    new WaitUntilCommand(slides::reachedTargetPosition),
                    new SetCurrentState(ArmState.IN_ROBOT)
            );
        }
        else if (currentState == ArmState.DEPOSIT){
            addCommands(
                    new SetClawState(TargetState.OPEN),
                    new WaitCommand(200),
                    new SetClawAngle(TargetAngle.DOWN),
                    new WaitCommand(100),
                    new SetClawState(TargetState.CLOSED_NORMAL),
                    new SetSlideExtension(ArmSlideConfiguration.TargetPosition.RETRACTED),
                    new WaitCommand(200),
                    new SetClawAngle(TargetAngle.DEPOSIT),
                    new WaitUntilCommand(slides::reachedTargetPosition),
                    new SetClawAngle(TargetAngle.UP),
                    new SetRotatorAngle(ArmRotatorConfiguration.TargetAngle.DOWN),
                    new WaitUntilCommand(arm::reachedTargetPosition),
                    new SetCurrentState(ArmState.IN_ROBOT)
            );
        }
        addRequirements(arm, slides);
    }
}
