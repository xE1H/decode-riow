package org.firstinspires.ftc.teamcode.auto.pedroCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.auto.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.pathGeneration.Point;

public class TranslateHeading extends CommandBase {
    static Follower follower;
    private final Point targetPoint;
    private boolean hasExecutedOnce = false;


    public TranslateHeading(Point targetPoint) {
        this.targetPoint = targetPoint;
    }

    public static void setFollower(Follower newFollower) {
        follower = newFollower;
    }

    private double calculateTargetHeading() {
        double currentY = follower.getPose().getY();
        double currentX = follower.getPose().getX() + 2;
        return Math.atan((targetPoint.getX() - currentX) / (targetPoint.getY() - currentY));
    }

    @Override
    public void execute() {
        if (!hasExecutedOnce) {
            hasExecutedOnce = true;
            double currentX = follower.getPose().getX();
            double currentY = follower.getPose().getY();

            double targetHeading = calculateTargetHeading();
            double currentHeading = follower.getTotalHeading();

            PathChain pathChain = follower.pathBuilder().addPath(new BezierLine(new Point(currentX, currentY), new Point(currentX + 2, currentY)))
                    .setLinearHeadingInterpolation(currentHeading, targetHeading)
                    .setPathEndHeadingConstraint(Math.toRadians(1))
                    .build();

            follower.followPath(pathChain);
        }
    }


    public boolean isFinished() {
        return !follower.isBusy();
    }
}
