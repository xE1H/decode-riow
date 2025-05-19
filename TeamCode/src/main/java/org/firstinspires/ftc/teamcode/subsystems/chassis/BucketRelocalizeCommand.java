package org.firstinspires.ftc.teamcode.subsystems.chassis;

import static org.firstinspires.ftc.teamcode.subsystems.chassis.ChassisConfiguration.BUCKET_CORNER;
import static org.firstinspires.ftc.teamcode.subsystems.chassis.ChassisConfiguration.DISTANCE_BETWEEN_ANGLED_SENSORS_MM;
import static org.firstinspires.ftc.teamcode.subsystems.chassis.ChassisConfiguration.OFFSET_FROM_SENSOR_MIDPOINT_TO_PEDRO_CENTER;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;

import org.firstinspires.ftc.teamcode.helpers.commands.InstantCommand;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;

import java.util.logging.Level;
import java.util.logging.Logger;

public class BucketRelocalizeCommand extends SequentialCommandGroup {
    private final double maxAllowedDistanceDelta = 2; //inches
    private final Logger logger = Logger.getLogger("BUCKET RELOCALIZE COMMAND");

    public BucketRelocalizeCommand(Follower follower){
        addCommands(
                new WaitCommand(20),
                new WaitUntilCommand(follower::atParametricEnd),
                new WaitCommand(1000),

                new InstantCommand() {
                    @Override
                    public void run() {
                        double leftDistance = VLRSubsystem.getInstance(Chassis.class).getLeftDistance();
                        double rightDistance = VLRSubsystem.getInstance(Chassis.class).getRightDistance();

                        Pose currentPose = follower.getPose();
                        double robotAngle = currentPose.getHeading();

                        logger.info("left distance: " + leftDistance);
                        logger.info("right distance: " + rightDistance);

                        double X_offset = rightDistance + 0.5 * DISTANCE_BETWEEN_ANGLED_SENSORS_MM * Math.sin(robotAngle);
                        double Y_offset = leftDistance + 0.5 * DISTANCE_BETWEEN_ANGLED_SENSORS_MM * Math.cos(robotAngle);

                        Pose midpointBetweenSensors = new Pose(BUCKET_CORNER.getX() + X_offset / 25.4, BUCKET_CORNER.getY() - Y_offset / 25.4, robotAngle);
                        Vector2d rotatedOffset = OFFSET_FROM_SENSOR_MIDPOINT_TO_PEDRO_CENTER.rotateBy(Math.toDegrees(robotAngle)).div(25.4);
                        logger.info("rotated offset: " + rotatedOffset.getX() + "; " + rotatedOffset.getY());

                        Pose calculatedPose = new Pose(midpointBetweenSensors.getX() + rotatedOffset.getX(), midpointBetweenSensors.getY() + rotatedOffset.getY(), robotAngle);

                        logger.info("CALCULATED POSITION FROM DISTANCE SENSORS IS: " + calculatedPose.getX() + "; " + calculatedPose.getY() + "; " + Math.toDegrees(calculatedPose.getHeading()));
                        logger.info("PEDRO ODOMETRY POSE IS: " + currentPose.getX() + "; " + currentPose.getY() + "; " + Math.toDegrees(currentPose.getHeading()));

                        double poseDeviation = Math.hypot(calculatedPose.getX() - currentPose.getX(), calculatedPose.getY() - currentPose.getY());

                        if (poseDeviation < maxAllowedDistanceDelta){
                            logger.info("POSE DEVIATION IS ACCEPTABLE, RELOCALIZING");
                            follower.resetOffset();
                            follower.setCurrentPoseWithOffset(calculatedPose);

                            currentPose = follower.getPose();
                            logger.info("CURRENT POSE IS: " + currentPose.getX() + "; " + currentPose.getY() + "; " + Math.toDegrees(currentPose.getHeading()));
                        }

                        else {logger.log(Level.SEVERE, "POSE DEVIATION IS UNACCEPTABLE, SKIPPING RELOCALIZATION");}
                    }
                }
        );
    }
}