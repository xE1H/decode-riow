package org.firstinspires.ftc.teamcode.subsystems.vision.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.vision.Vision;

/**
 * Processes one frame using the machine learning model.
 */
public class ProcessFrame extends CommandBase {
    Vision vision;

    @Override
    public void initialize() {
        vision = VLRSubsystem.getInstance(Vision.class);
    }

    @Override
    public void execute() {
        vision.setEnabled(true);
    }

    @Override
    public void end(boolean interrupted) {
        vision.setEnabled(false);
    }

    @Override
    public boolean isFinished() {
        return vision.isFrameProcessed();
    }
}
