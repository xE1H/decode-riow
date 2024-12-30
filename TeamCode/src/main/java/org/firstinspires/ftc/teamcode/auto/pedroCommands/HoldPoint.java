package org.firstinspires.ftc.teamcode.auto.pedroCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.auto.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.pathGeneration.BezierPoint;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.pathGeneration.Point;

public class HoldPoint extends CommandBase {
    private static Follower follower;
    private final BezierPoint bezierPoint;
    private final double heading;
    private final boolean isPerpetual;

    public HoldPoint(Pose pose, boolean isPerpetual){
        bezierPoint = new BezierPoint(new Point(pose));
        heading = pose.getHeading();
        this.isPerpetual = isPerpetual;
    }

    public HoldPoint(Pose pose){
        this(pose, false);
    }

    public static void setFollower(Follower newFollower){
        follower = newFollower;
    }

    @Override
    public void initialize() {
        follower.holdPoint(bezierPoint, Math.toRadians(heading));
    }

    @Override
    public void execute() {
        follower.update();
    }

    @Override
    public boolean isFinished() {
        return !isPerpetual;
    }
}
