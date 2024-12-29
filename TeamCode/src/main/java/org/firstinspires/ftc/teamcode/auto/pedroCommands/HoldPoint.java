package org.firstinspires.ftc.teamcode.auto.pedroCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auto.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.pathGeneration.BezierPoint;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.pathGeneration.Point;

public class HoldPoint extends CommandBase {
    private final Follower m_follower;
    private final BezierPoint m_point;
    private final double m_heading;
    private ElapsedTime timer;
    private double millis;

    public HoldPoint(Follower follower, BezierPoint point, double heading, double millis) {
        m_follower = follower;
        m_point = point;
        m_heading = heading;
        this.millis = millis;
    }

    public HoldPoint(Follower follower, Pose pose, double millis){
        m_follower = follower;
        m_point = new BezierPoint(new Point(pose));
        m_heading = pose.getHeading();
        this.millis = millis;
    }

    @Override
    public void initialize() {
        m_follower.holdPoint(m_point, m_heading);
        timer = new ElapsedTime();
    }

    @Override
    public void execute() {
        m_follower.update();
    }

    @Override
    public boolean isFinished() {
        return timer.milliseconds() > millis;
    }
}
