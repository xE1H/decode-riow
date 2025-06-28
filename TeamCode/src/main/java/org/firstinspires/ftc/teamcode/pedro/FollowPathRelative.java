package org.firstinspires.ftc.teamcode.pedro;

import static org.firstinspires.ftc.teamcode.helpers.pedro.PoseToPath.bezierPath;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.commands.FollowPath;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;

import org.firstinspires.ftc.teamcode.helpers.commands.InstantCommand;

import java.util.logging.Logger;

@Config
public class FollowPathRelative extends SequentialCommandGroup {
    private final Logger logger = Logger.getLogger("Follow path relative");

    private final SequentialCommandGroup moveRelativeCommand = new SequentialCommandGroup();

    public FollowPathRelative(Follower follower, Pose delta) {
        addCommands(
                new InstantCommand() {
                    @Override
                    public void run() {
                        Pose currentPose = follower.getPose();
                        Pose target = new Pose(currentPose.getX() + delta.getX(), currentPose.getY() + delta.getY(), currentPose.getHeading() + delta.getHeading());

                        moveRelativeCommand.addCommands(
                                new FollowPath(follower, bezierPath(currentPose, target)
                                        .setLinearHeadingInterpolation(currentPose.getHeading(), target.getHeading()).setPathEndTValueConstraint(0.9).build())
                        );
                    }
                },
                moveRelativeCommand
        );

    }
}
