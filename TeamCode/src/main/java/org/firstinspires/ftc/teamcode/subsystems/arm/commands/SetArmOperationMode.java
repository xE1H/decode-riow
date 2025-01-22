package org.firstinspires.ftc.teamcode.subsystems.arm.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideSubsystem;

public class SetArmOperationMode extends InstantCommand {
    public SetArmOperationMode(ArmSlideConfiguration.OperationMode operationMode){
        super(()-> VLRSubsystem.getInstance(ArmSlideSubsystem.class).setOperationMode(operationMode));
    }
}
