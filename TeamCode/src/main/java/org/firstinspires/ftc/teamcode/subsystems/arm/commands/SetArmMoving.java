package org.firstinspires.ftc.teamcode.subsystems.arm.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.helpers.commands.InstantCommand;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmState;

public class SetArmMoving extends InstantCommand {
    @Override
    public void run() {
        ArmState.setMoving(true);
    }
}
