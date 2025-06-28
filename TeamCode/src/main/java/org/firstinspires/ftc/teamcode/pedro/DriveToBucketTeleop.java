//package org.firstinspires.ftc.teamcode.pedro;
//
//import static org.firstinspires.ftc.teamcode.auto.sample.PointsSample.BUCKET_HIGH_SCORE_POSE;
//import static org.firstinspires.ftc.teamcode.helpers.pedro.PoseToPath.bezierPath;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.arcrobotics.ftclib.command.SequentialCommandGroup;
//import com.pedropathing.commands.FollowPath;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.localization.Pose;
//
//import org.firstinspires.ftc.teamcode.helpers.commands.InstantCommand;
//
//import java.util.logging.Logger;
//
//@Config
//public class DriveToBucketTeleop extends SequentialCommandGroup {
//    private final Logger logger = Logger.getLogger("Follow path relative");
//    private double xBound = 10;
//
//    private final SequentialCommandGroup moveRelativeCommand = new SequentialCommandGroup();
//
//    public DriveToBucketTeleop(Follower follower) {
//        addCommands(
//                new InstantCommand() {
//                    @Override
//                    public void run() {
//                        Pose currentPose = follower.getPose();
//                        Pose target = BUCKET_HIGH_SCORE_POSE;
//
//                        if (currentPose.getX() > )
//
//                        moveRelativeCommand.addCommands(
//                                new FollowPath(follower, bezierPath(currentPose, target)
//                                        .setLinearHeadingInterpolation(currentPose.getHeading(), target.getHeading()).setPathEndTValueConstraint(0.9).build())
//                        );
//                    }
//                },
//                moveRelativeCommand
//        );
//
//    }
//}
