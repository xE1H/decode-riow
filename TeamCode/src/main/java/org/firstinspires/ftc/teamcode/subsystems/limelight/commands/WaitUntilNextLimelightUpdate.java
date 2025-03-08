package org.firstinspires.ftc.teamcode.subsystems.limelight.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.limelight.Limelight;

public class WaitUntilNextLimelightUpdate extends CommandBase {
    @Override
    public boolean isFinished() {
        if (VLRSubsystem.getInstance(Limelight.class).getTimeSinceLastUpdate() < 100)
            return true;

        return false;
    }
}
