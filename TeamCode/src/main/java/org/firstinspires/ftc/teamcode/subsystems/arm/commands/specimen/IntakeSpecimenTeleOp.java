package org.firstinspires.ftc.teamcode.subsystems.arm.commands.specimen;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.helpers.commands.CustomConditionalCommand;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmState;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.RetractArm;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawState;

@Config
public class IntakeSpecimenTeleOp extends SequentialCommandGroup {

    public IntakeSpecimenTeleOp() {
        addCommands(
                new CustomConditionalCommand(
                        new PrepareSpecimenIntake(),
                        new CustomConditionalCommand(
                                new SequentialCommandGroup(
                                        new SetClawState(ClawConfiguration.GripperState.CLOSED),
                                        new WaitCommand(100),
                                        new RetractArm()
                                ),
                                () -> ArmState.isCurrentState(ArmState.State.SPECIMEN_INTAKE)
                        ),
                        () -> ArmState.isCurrentState(ArmState.State.IN_ROBOT)
                )
        );
    }
}
