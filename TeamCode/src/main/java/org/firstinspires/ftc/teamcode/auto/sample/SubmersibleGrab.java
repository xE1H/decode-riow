package org.firstinspires.ftc.teamcode.commands.sample;

import static org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration.MAX_POSITION;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.helpers.commands.InstantCommand;
import org.firstinspires.ftc.teamcode.helpers.commands.LogCommand;
import org.firstinspires.ftc.teamcode.helpers.controls.rumble.RumbleControls;
import org.firstinspires.ftc.teamcode.helpers.enums.Alliance;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmState;
import org.firstinspires.ftc.teamcode.subsystems.arm.SetArmPosition;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawAngle;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawState;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawTwist;
import org.firstinspires.ftc.teamcode.subsystems.limelight.LimelightYoloReader;

import java.util.logging.Level;
import java.util.logging.Logger;

public class SubmersibleGrab extends SequentialCommandGroup {

    private SequentialCommandGroup submersibleGrabCommand = new SequentialCommandGroup();

    double angle = 90;

    public SubmersibleGrab(Follower f, Alliance alliance, LimelightYoloReader reader, RumbleControls rc) {
        Logger logger = Logger.getLogger("SubmersibleGrab");
        addCommands(
                new LogCommand("SubmersibleGrab", Level.INFO, "Sub grab command"),
                new ParallelCommandGroup(
                        //new WaitUntilNextLimelightUpdate(),
                        new SequentialCommandGroup(
                                new SetClawState(ClawConfiguration.GripperState.OPEN),
                                new SetClawAngle(ClawConfiguration.VerticalRotation.DEPOSIT),
                                new SetClawTwist(ClawConfiguration.HorizontalRotation.NORMAL)
                        )
                ),
                new InstantCommand() {
                    @Override
                    public void run() {
                        LimelightYoloReader.Limelight.Sample sample = reader.getBestSampleWithRetry();
                        if (sample == null) {
                            logger.info("Found nothing ts pmo");
                            if (rc != null) rc.doubleBlip();

                            //generateRetry(f, alliance);
                        } else {
                            angle = Math.toDegrees(sample.getAngle());
                            if (angle == -360) {
                                logger.info("Found nothing ts pmo");
                                if (rc != null) rc.doubleBlip();
                                return;
                            }
                            if (angle < 0) angle += 180;

                            logger.info("Going for sample: " + sample.getColor() + " in X: " + sample.getX() + " Y: " + sample.getY());
                            logger.info("Sample angle: " + angle);

                            generateSubmersibleGrabCommand(f, sample);
                        }
                    }
                },
                submersibleGrabCommand,
                new WaitCommand(200),
                new SetArmPosition().retract()
        );

    }

//    private void generateRetry(Follower f, Alliance alliance) {
//        if (ScheduleNext.XGrabOffset >= 20) {
//            ScheduleNext.XGrabOffsetMaxTriggered = true;
//        } else if (ScheduleNext.XGrabOffsetMaxTriggered && ScheduleNext.XGrabOffset == 0) {
//            ScheduleNext.XGrabOffsetMaxTriggered = false;
//        }
//        ScheduleNext.XGrabOffset += 5 * (ScheduleNext.XGrabOffsetMaxTriggered ? -1 : 1); // Do not go to the same spot again if left it to go looking before
//        submersibleGrabCommand.addCommands(
//                new MoveRelative(f, 5 * (ScheduleNext.XGrabOffsetMaxTriggered ? -1 : 1), 0),
//                new WaitCommand(1000),
//                new SubmersibleGrab(f, alliance)
//        );
//    }

    private void generateSubmersibleGrabCommand(Follower f, LimelightYoloReader.Limelight.Sample sample) {
        double x_abs = f.getPose().getX() + sample.getY() * Math.cos(f.getPose().getHeading()) - sample.getX() * Math.sin(f.getPose().getHeading());
        double y_abs = f.getPose().getY() + sample.getY() * Math.sin(f.getPose().getHeading()) + sample.getX() * Math.cos(f.getPose().getHeading());

        submersibleGrabCommand.addCommands(
                new ParallelCommandGroup(
                        new MoveRelative(f, -sample.getX(), 0),
                        new SetArmPosition().extensionAndAngleDegrees((0.7742 * (sample.getY() + 1.5)) / MAX_POSITION, 2),
                        new SequentialCommandGroup(
                                new SetClawAngle(ClawConfiguration.VerticalRotation.DOWN),
                                new WaitCommand(450),
                                new SetClawTwist((angle / -90) + 1),
                                new SetArmPosition().angleDegrees(0)
                                //new SetClawTwist(isVerticallyOriented ? ClawConfiguration.HorizontalRotation.NORMAL : ClawConfiguration.HorizontalRotation.FLIPPED),
                        )
                ),
                new WaitCommand(250),
                new SetClawState(ClawConfiguration.GripperState.CLOSED),
                new WaitCommand(150)
        );
    }


    public SubmersibleGrab(Follower f, Alliance alliance, LimelightYoloReader reader) {
        this(f, alliance, reader, null);
    }
}
