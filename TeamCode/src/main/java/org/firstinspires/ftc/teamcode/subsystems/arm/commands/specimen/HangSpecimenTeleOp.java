package org.firstinspires.ftc.teamcode.subsystems.arm.commands.specimen;

//importynas
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.helpers.commands.CustomConditionalCommand;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmState;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.RetractArm;

@Config
public class HangSpecimenTeleOp extends SequentialCommandGroup {

    public HangSpecimenTeleOp() {
        addCommands(
                new CustomConditionalCommand(
                        new SequentialCommandGroup(
                                new ScoreSpecimenHigh(),
                                new WaitCommand(200),
                                new RetractArm()
                        ),
                        new CustomConditionalCommand(
                                new PrepareSpecimenHigh(),
                                () -> ArmState.isCurrentState(ArmState.State.IN_ROBOT)
                        ),
                        () -> ArmState.isCurrentState(ArmState.State.SPECIMEN_PREPARE)
                )
        );
    }

}
