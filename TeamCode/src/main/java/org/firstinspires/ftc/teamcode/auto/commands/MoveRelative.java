package org.firstinspires.ftc.teamcode.auto.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.auto.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.pathGeneration.Point;

public class MoveRelative extends CommandBase {
    private static Follower follower;
    private PathChain pathChain = null;

    public static void setFollower(Follower newFollower) {
        follower = newFollower;
    }

    public MoveRelative(double relativeX, double relativeY) {
        pathChain = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(follower.getPose().getX(), follower.getPose().getY()),
                                new Point(follower.getPose().getX() + relativeX, follower.getPose().getY() + relativeY)
                        )
                ).setConstantHeadingInterpolation(follower.getPose().getHeading())
                .setPathEndTranslationalConstraint(0.1)
                .setPathEndHeadingConstraint(Math.PI / (360 * 2))
                .setPathEndTValueConstraint(0.999)
                .build();
    }

    public void initialize() {
        follower.followPath(pathChain);
    }

    @Override
    public boolean isFinished() {
        return !follower.isBusy();
    }
}
