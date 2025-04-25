//package org.firstinspires.ftc.teamcode.commands.sample;
//
//import static org.firstinspires.ftc.teamcode.auto.sample.Points_sample.BUCKET_HIGH_SCORE_POSE;
//import static org.firstinspires.ftc.teamcode.auto.sample.Points_sample.SUB_GRAB_POSE;
//import static org.firstinspires.ftc.teamcode.auto.sample.Points_sample.SUB_PRE_BEZIER_POSE;
//import static org.firstinspires.ftc.teamcode.helpers.pedro.PoseToPath.bezierPath;
//
//import com.arcrobotics.ftclib.command.CommandBase;
//import com.arcrobotics.ftclib.command.ParallelCommandGroup;
//import com.arcrobotics.ftclib.command.SequentialCommandGroup;
//import com.arcrobotics.ftclib.command.WaitCommand;
//import com.pedropathing.commands.FollowPath;
//import com.pedropathing.follower.Follower;
//
//import org.firstinspires.ftc.teamcode.subsystems.arm.ArmState;
//import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetCurrentArmState;
//import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetSlideExtension;
//import org.firstinspires.ftc.teamcode.subsystems.arm.commands.sample.ScoreSample;
//import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration;
//import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
//import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawAngle;
//import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawTwist;
//
//public class HighBasketScore extends SequentialCommandGroup {
//    public static int times = 0;
//
//    public HighBasketScore(Follower f, CommandBase allowDriving) {
//        times++;
//        addCommands(
//                new ParallelCommandGroup(
//                        new SetClawTwist(ClawConfiguration.HorizontalRotation.NORMAL),
//                        new SetClawAngle(ClawConfiguration.VerticalRotation.SAFE),
//                        new SetSlideExtension(ArmSlideConfiguration.TargetPosition.RETRACTED),
//                        new FollowPath(f, bezierPath(f.getPose(), SUB_PRE_BEZIER_POSE, BUCKET_HIGH_SCORE_POSE)//new Pose(SUB_GRAB_POSE.getX() + ScheduleNext.XGrabOffset, SUB_GRAB_POSE.getY(), SUB_GRAB_POSE.getHeading()), SUB_PRE_BEZIER_POSE, BUCKET_HIGH_SCORE_POSE)
//                                .setLinearHeadingInterpolation(SUB_GRAB_POSE.getHeading(), BUCKET_HIGH_SCORE_POSE.getHeading())
//                                .build()), // drive to the high basket
//                        new SequentialCommandGroup(
//                                new SetCurrentArmState(ArmState.State.IN_ROBOT),
//                                new WaitCommand(1600),
//                                new ParallelCommandGroup(
//                                        allowDriving,
//                                        new ScoreSample(110)
////                                        new ConditionalCommand(
////                                                new SequentialCommandGroup(
////                                                        new WaitCommand(700),
////                                                        new FollowPath(f, bezierPath(BUCKET_HIGH_SCORE_POSE, new Pose(BUCKET_HIGH_SCORE_POSE.getX() - 3, BUCKET_HIGH_SCORE_POSE.getY() + 3, BUCKET_HIGH_SCORE_POSE.getHeading())).setConstantHeadingInterpolation(BUCKET_HIGH_SCORE_POSE.getHeading()).build()),
////                                                        new FollowPath(f, bezierPath(new Pose(BUCKET_HIGH_SCORE_POSE.getX() - 3, BUCKET_HIGH_SCORE_POSE.getY() + 3, BUCKET_HIGH_SCORE_POSE.getHeading()), BUCKET_HIGH_SCORE_POSE).setConstantHeadingInterpolation(BUCKET_HIGH_SCORE_POSE.getHeading()).build())
////                                                        ),
////                                                new SequentialCommandGroup(),
////                                                () -> times % 7 == 0
////                                        )
//                                )
//                        )
//                )
//        );
//    }
//}
