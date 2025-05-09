package org.firstinspires.ftc.teamcode.helpers.utils.opmodes;

import static org.firstinspires.ftc.teamcode.auto.sample.PointsSample.rad;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.outoftheboxrobotics.photoncore.Photon;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auto.sample.SubmersibleGrabV2;
import org.firstinspires.ftc.teamcode.helpers.opmode.VLRLinearOpMode;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmState;
import org.firstinspires.ftc.teamcode.subsystems.arm.MainArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.SetArmPosition;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.limelight.LimelightYoloReader;

@TeleOp(name = "LimelightAimer", group = "Utils")
@Photon
@Config
public class LimelightAimerTest extends VLRLinearOpMode {

    public static boolean go = false;

    @Override
    public void run() {

        VLRSubsystem.requireSubsystems(MainArmSubsystem.class, ClawSubsystem.class);
        VLRSubsystem.initializeAll(hardwareMap);

        Follower f = new Follower(hardwareMap, pedroPathing.tuners.constants.FConstants.class, pedroPathing.tuners.constants.LConstants.class);
        f.setStartingPose(new Pose(0, 0, rad(0)));
        CommandScheduler cs = CommandScheduler.getInstance();
        LimelightYoloReader reader = new LimelightYoloReader();


        waitForStart();

//        while (!go) {
//            sleep(10);
//        }

        cs.schedule(new SequentialCommandGroup(
                new SetArmPosition().setArmState(ArmState.State.IN_ROBOT),
                new WaitCommand(200),
                new SubmersibleGrabV2(f, reader),
                new SetArmPosition().retract()
        ));

        while (opModeIsActive()) {
            f.update();
        }
    }
}