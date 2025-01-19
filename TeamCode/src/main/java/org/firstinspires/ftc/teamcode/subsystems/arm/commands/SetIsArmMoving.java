package org.firstinspires.ftc.teamcode.subsystems.arm.commands;

import org.firstinspires.ftc.teamcode.helpers.commands.InstantCommand;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmState;

public class SetIsArmMoving extends InstantCommand {
    @Override
    public void run() {
        ArmState.setMoving(true);
    }
}
