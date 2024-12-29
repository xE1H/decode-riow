//package org.firstinspires.ftc.teamcode.testOpModes;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.arcrobotics.ftclib.command.CommandScheduler;
//import com.arcrobotics.ftclib.command.SequentialCommandGroup;
//import com.outoftheboxrobotics.photoncore.Photon;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//
//import org.firstinspires.ftc.teamcode.auto.pedroCommands.FollowPath;
//import org.firstinspires.ftc.teamcode.auto.pedroCommands.HoldPoint;
//import org.firstinspires.ftc.teamcode.auto.pedroPathing.follower.Follower;
//import org.firstinspires.ftc.teamcode.auto.pedroPathing.localization.Pose;
//import org.firstinspires.ftc.teamcode.auto.pedroPathing.pathGeneration.BezierLine;
//import org.firstinspires.ftc.teamcode.auto.pedroPathing.pathGeneration.Point;
//import org.firstinspires.ftc.teamcode.helpers.opmode.VLRLinearOpMode;
//
//@Config
//@Photon
//@Autonomous(name = "AutoPointTuner")
//public class AutoPointTuner extends VLRLinearOpMode {
//    private Follower follower;
//
//    public static double targetX = 9.6;
//    public static double targetY = 63.2;
//    public static double targetHeading = -180;
//
//
//    private Pose targetPose = new Pose(targetX, targetY, Math.toRadians(targetHeading));
//    private Pose prevPose = targetPose;
//    private double prevHeading = targetHeading;
//
//    @Override
//    public void run() {
//        follower = new Follower(hardwareMap);
//        follower.setStartingPose(targetPose);
//        FollowPath.setStartingPoint(new Point(targetX, targetY));
//        FollowPath.setFollower(follower);
//        follower.setMaxPower(0.4);
//        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
//
//
//        waitForStart();
//        CommandScheduler.getInstance().schedule(
//                new FollowPath(
//                        targetHeading,
//                        new Point(prevPose), new Point(targetPose)
//                ),
//                new HoldPoint(
//                        follower,
//                        targetPose,
//                        100
//                )
//        );
//
//
//        while (opModeIsActive()) {
//            targetPose = new Pose(targetX, targetY, Math.toRadians(targetHeading));
//
//            if (!prevPose.roughlyEquals(targetPose)) {
//
//                FollowPath followPath;
//                if(targetHeading == prevHeading){
//                    followPath = new FollowPath(
//                            targetHeading,
//                            new Point(prevPose), new Point(targetPose)
//                    );
//                } else {
//                    followPath = new FollowPath(
//                            prevHeading,
//                            targetHeading,
//                            new Point(prevPose), new Point(targetPose)
//                    );
//                }
//
//                CommandScheduler.getInstance().schedule(
//                        new SequentialCommandGroup(
//                                followPath,
//                                new HoldPoint(
//                                        follower,
//                                        targetPose,
//                                        100
//                                )
//                        )
//
//                );
//                prevPose = new Pose(targetX, targetY, Math.toRadians(targetHeading));
//                prevHeading = targetHeading;
//            }
//
////            if(follower.getPose() != null){
////                follower.telemetryDebug(telemetry);
////            }
//            telemetry.update();
//        }
//    }
//}