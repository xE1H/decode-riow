package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Points_specimen.START_POSE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.outoftheboxrobotics.photoncore.Photon;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.commands.specimen.AutonomousPeriodActions;
import org.firstinspires.ftc.teamcode.commands.specimen.AutonomousPeriodActionsBetter;
import org.firstinspires.ftc.teamcode.helpers.opmode.VLRLinearOpMode;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@TeleOp(name = "VLR_SpecimenAuto", group = "!TELEOP")
@Photon
public class VLRSpecimenAuto extends VLRLinearOpMode {
    CommandScheduler cs;
    Follower f;

    public static double velocityThreshold = 9999999;
    public static double headingThreshold = 9999999;
    public static double driveThreshold = 9999999;
    public static double translationalThreshold = 99999999;

    /**
     * @noinspection unchecked
     */
    @Override
    public void run() {
        Constants.setConstants(FConstants.class, LConstants.class);

        cs = CommandScheduler.getInstance();

        VLRSubsystem.requireSubsystems(ArmSlideSubsystem.class, ArmRotatorSubsystem.class, ClawSubsystem.class);
        VLRSubsystem.initializeAll(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        f = new Follower(hardwareMap);
        f.setStartingPose(START_POSE);

        cs.schedule(new AutonomousPeriodActionsBetter(f));

        while (opModeIsActive()) {
            f.update();

            telemetry.addData("FOLLOWER VELOCITY: ", Math.abs(f.getVelocityMagnitude()));
            telemetry.addData("FOLLOWER HEADING ERROR: ", Math.abs(f.headingError));
            telemetry.addData("FOLLOWER POSITION ERROR: ", Math.abs(f.driveError));
            telemetry.addData("FOLLOWER TRANSLATIONAL ERROR", Math.abs(f.getTranslationalError().getMagnitude()));

            boolean settled = Math.abs(f.getVelocityMagnitude()) < velocityThreshold && Math.abs(f.headingError) < headingThreshold &&
                    Math.abs(f.driveError) < driveThreshold && Math.abs(f.getTranslationalError().getMagnitude()) < translationalThreshold;
            telemetry.addData("SETTLED: ", settled ? 1 : 0);

            telemetry.update();
        }
    }
}
