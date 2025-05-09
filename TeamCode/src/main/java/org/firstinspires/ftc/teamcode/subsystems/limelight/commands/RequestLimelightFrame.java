package org.firstinspires.ftc.teamcode.subsystems.limelight.commands;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.helpers.commands.InstantCommand;
import org.firstinspires.ftc.teamcode.subsystems.limelight.LimelightYoloReader;

public class RequestLimelightFrame extends InstantCommand {
    LimelightYoloReader reader;
    Follower f;

    public RequestLimelightFrame(LimelightYoloReader reader) {
        this.reader = reader;
    }

    public RequestLimelightFrame(LimelightYoloReader reader, Follower follower) {
        this.reader = reader;
        this.f = follower;
    }

    @Override
    public void run() {
        reader.requestFrame();
        if (f != null) {
            reader.setFollowerFramePose(f.getPose());
        } else {
            reader.setFollowerFramePose(null);
        }
    }
}
