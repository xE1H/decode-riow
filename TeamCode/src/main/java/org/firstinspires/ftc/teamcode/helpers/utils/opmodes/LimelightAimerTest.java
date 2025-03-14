package org.firstinspires.ftc.teamcode.helpers.utils.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.outoftheboxrobotics.photoncore.Photon;
import com.pedropathing.follower.Follower;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Points_sample;
import org.firstinspires.ftc.teamcode.helpers.enums.Alliance;
import org.firstinspires.ftc.teamcode.helpers.opmode.VLRLinearOpMode;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.limelight.Limelight;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@TeleOp(name = "LimelightAimer", group = "Utils")
@Photon
@Config
public class LimelightAimerTest extends VLRLinearOpMode {

    public static boolean go = false;

    @Override
    public void run() {
        Constants.setConstants(FConstants.class, LConstants.class);

        VLRSubsystem.requireSubsystems(Limelight.class, ArmSlideSubsystem.class, ArmRotatorSubsystem.class, ClawSubsystem.class);
        VLRSubsystem.initializeAll(hardwareMap);

        Follower f = new Follower(hardwareMap);
        f.setStartingPose(Points_sample.SUB_GRAB_POSE);
        CommandScheduler cs = CommandScheduler.getInstance();
        Limelight ll = VLRSubsystem.getInstance(Limelight.class);

        waitForStart();
        ll.setAlliance(Alliance.BLUE);
        ll.enable();

        while (!go) {
            sleep(10);
        }

//        cs.schedule(new SubmersibleGrab(f, Alliance.BLUE));

        while (opModeIsActive()) {
            f.update();
        }
    }
}