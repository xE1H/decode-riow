package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.auto.pedroCommands.FollowPath;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.helpers.opmode.VLRLinearOpMode;
import org.firstinspires.ftc.teamcode.helpers.utils.GlobalConfig;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmState;

@Config
@Photon
@Autonomous(name = "VLRAuto")
public class VLRAuto extends VLRLinearOpMode {
    private Follower follower;

    private final double xStart = 9.6;
    private final double yStart = 60;


    @Override
    public void run() {

        GlobalConfig.setIsInvertedMotors(true);

        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(xStart, yStart, 0));
        follower.setMaxPower(0.6);

        FollowPath.setStartingPoint(new Point(xStart, yStart));
        FollowPath.setFollower(follower);

        waitForStart();
        schedulePath();

        CommandScheduler.getInstance().run();

        while (opModeIsActive()) {
            follower.telemetryDebug(FtcDashboard.getInstance().getTelemetry());
        }
    }


    private void schedulePath() {
        CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
                new FollowPath(0, new Point(28, 60)),
                new FollowPath(0, -90, new Point(28.05, 60)),
                new WaitCommand(500),

                new FollowPath(false,
                        new Point(29, 42.5),
                        new Point(38.5, 29),
                        new Point(57.5, 29)),

                new WaitCommand(50),

                new FollowPath(0, new Point(57.5, 19)),
                new FollowPath(0, new Point(21, 19)),
                new FollowPath(0, new Point(57.5, 19)),
                new FollowPath(0, new Point(57, 9.5)),
                new FollowPath(0, new Point(21, 9.5)),

                new FollowPath(0, new Point(57, 9.5)),
                new FollowPath(0, new Point(57, 3.5)),
                new FollowPath(0, new Point(21, 3.5))
        ));
    }

}
