package org.firstinspires.ftc.teamcode.helpers.testOpmodes;

import static org.firstinspires.ftc.teamcode.auto.sample.PointsSample.BUCKET_HIGH_SCORE_POSE;
import static org.firstinspires.ftc.teamcode.auto.sample.PointsSample.START_POSE;
import static org.firstinspires.ftc.teamcode.helpers.pedro.PoseToPath.bezierPath;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.outoftheboxrobotics.photoncore.Photon;
import com.pedropathing.commands.FollowPath;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.helpers.opmode.VLRAutoTestOpMode;
import org.firstinspires.ftc.teamcode.subsystems.chassis.BucketRelocalizeCommand;
import org.firstinspires.ftc.teamcode.subsystems.limelight.LimelightYoloReader;


@Autonomous(name = "SensorRelocalizationTest")
@Photon
public class RangefinderSensorRelocalizationTest extends VLRAutoTestOpMode {
    @Override
    public Pose StartPose() {
        return START_POSE;
    }

    @Override
    public Command autoCommand(Follower f, LimelightYoloReader reader) {
        return new ParallelCommandGroup(
                new FollowPath(f, bezierPath(START_POSE, BUCKET_HIGH_SCORE_POSE)
                        .setLinearHeadingInterpolation(START_POSE.getHeading(), BUCKET_HIGH_SCORE_POSE.getHeading()).build()
                ),
                new BucketRelocalizeCommand(f)
        );
//        return new InstantCommand();
    }

    @Override
    public boolean SpecimenOnly(){
        return false;
    }
}
