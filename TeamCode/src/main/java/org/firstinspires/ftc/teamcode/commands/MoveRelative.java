package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.helpers.pedro.PoseToPath.bezierPath;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.commands.FollowPath;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;

public class MoveRelative extends SequentialCommandGroup {
    public MoveRelative(Follower f, double relativeX, double relativeY) {
        PathBuilder path = bezierPath(f.getPose(), new Pose(f.getPose().getX() + relativeX, f.getPose().getY() + relativeY, f.getPose().getHeading()))
                .setConstantHeadingInterpolation(f.getPose().getHeading())
                .setPathEndTranslationalConstraint(0.1)
                .setPathEndHeadingConstraint(Math.PI / (360 * 2))
                .setPathEndTValueConstraint(0.999);

        addCommands(new FollowPath(f, path.build()));
    }
}
