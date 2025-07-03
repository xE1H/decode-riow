package org.firstinspires.ftc.teamcode.pedro;

import static org.firstinspires.ftc.teamcode.helpers.pedro.PoseToPath.bezierPath;
import static org.firstinspires.ftc.teamcode.subsystems.arm.MainArmSubsystem.mapToRange;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.commands.FollowPath;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import org.firstinspires.ftc.teamcode.helpers.commands.InstantCommand;
import java.util.logging.Level;
import java.util.logging.Logger;
import static org.firstinspires.ftc.teamcode.auto.specimen.PointsSpecimen.HUMAN_PLAYER_SPEC_DEPOSIT;

public class DriveToHumanPlayerTeleop extends SequentialCommandGroup {
    private final Logger logger = Logger.getLogger("Drive to human player zone teleop");
    private final double xBoundLow = 48;
    private final double xBoundHigh = 94;
    private final double xBound = 87;
    private final double yLowBound = 56;
    private final double yHighBound = 82;
    private final double xClose = 38.5;

    private final SequentialCommandGroup driveToBucketCommand = new SequentialCommandGroup();

    public DriveToHumanPlayerTeleop(Follower follower) {
        addCommands(
                new InstantCommand() {
                    @Override
                    public void run() {
                        Pose currentPose = follower.getPose();
                        if (currentPose.getX() > xBoundLow){
                            if (currentPose.getY() > yHighBound){
                                logger.info("robot is in the left sub sector, skipping");
                            }

                            else if (currentPose.getY() < yLowBound){
                                logger.info("robot is in the right sub sector");

                                double delta = mapToRange(currentPose.getX(), xBoundLow, xBoundHigh, 9.5, 3);
                                Pose target = new Pose(currentPose.getX(), currentPose.getY() - delta, Math.toRadians(65));

                                if (currentPose.getX() > xBound) {target = new Pose(target.getX() - 3, target.getY(), target.getHeading());}

                                driveToBucketCommand.addCommands(
                                        new FollowPath(follower, bezierPath(currentPose, target)
                                                .setLinearHeadingInterpolation(currentPose.getHeading(), target.getHeading()).setZeroPowerAccelerationMultiplier(6.5).setPathEndTValueConstraint(0.92).build(), false),

                                        new FollowPath(follower, bezierPath(target, HUMAN_PLAYER_SPEC_DEPOSIT).setZeroPowerAccelerationMultiplier(2.75)
                                                .setLinearHeadingInterpolation(target.getHeading(), HUMAN_PLAYER_SPEC_DEPOSIT.getHeading()).setPathEndTValueConstraint(0.6).build())
                                );
                            }
                            else{
                                logger.log(Level.SEVERE, "ROBOT APPEARS TO BE INSIDE THE SUBMERSIBLE");
                            }
                        }
                        else{
                            logger.info("robot is in the front sub sector");
                            Pose target = new Pose(xClose, currentPose.getY(), 0);

                            if (currentPose.getX() > xClose) {
                                driveToBucketCommand.addCommands(
                                        new FollowPath(follower, bezierPath(currentPose, target)
                                                .setLinearHeadingInterpolation(currentPose.getHeading(), target.getHeading()).setZeroPowerAccelerationMultiplier(6.5).setPathEndTValueConstraint(0.75).build(), false),

                                        new FollowPath(follower, bezierPath(target, HUMAN_PLAYER_SPEC_DEPOSIT).setZeroPowerAccelerationMultiplier(5)
                                                .setConstantHeadingInterpolation(HUMAN_PLAYER_SPEC_DEPOSIT.getHeading()).setPathEndTValueConstraint(0.6).build())
                                );
                            }
                            else{
                                driveToBucketCommand.addCommands(
                                        new FollowPath(follower, bezierPath(currentPose, HUMAN_PLAYER_SPEC_DEPOSIT).setZeroPowerAccelerationMultiplier(5)
                                                .setLinearHeadingInterpolation(currentPose.getHeading(), HUMAN_PLAYER_SPEC_DEPOSIT.getHeading()).setPathEndTValueConstraint(0.6).build())
                                );
                            }
                        }
                    }
                },
                driveToBucketCommand
        );

    }
}