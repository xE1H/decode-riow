package org.firstinspires.ftc.teamcode.helpers.utils.opmodes;

import static org.firstinspires.ftc.teamcode.auto.sample.PointsSample.rad;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.outoftheboxrobotics.photoncore.Photon;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Pedro.constants.FConstants;
import org.firstinspires.ftc.teamcode.Pedro.constants.LConstants;
import org.firstinspires.ftc.teamcode.auto.sample.SubmersibleGrabV2;
import org.firstinspires.ftc.teamcode.helpers.commands.RepeatUntilCommand;
import org.firstinspires.ftc.teamcode.helpers.opmode.VLRLinearOpMode;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmState;
import org.firstinspires.ftc.teamcode.subsystems.arm.MainArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.SetArmPosition;
import org.firstinspires.ftc.teamcode.subsystems.blinkin.BlinkinSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.limelight.LimelightYoloReader;

@TeleOp(name = "LimelightAimer", group = "Utils")
@Photon
@Config
public class LimelightAimerTest extends VLRLinearOpMode {

    public static boolean go = false;

    @Override
    public void run() {
        VLRSubsystem.requireSubsystems(MainArmSubsystem.class, ClawSubsystem.class, BlinkinSubsystem.class);
        VLRSubsystem.initializeAll(hardwareMap);

        Follower f = new Follower(hardwareMap, FConstants.class, LConstants.class);
        f.setStartingPose(new Pose(0, 0, rad(0)));
        CommandScheduler cs = CommandScheduler.getInstance();
        LimelightYoloReader reader = new LimelightYoloReader();
        ArmState.resetAll();


        waitForStart();

//        while (!go) {
//            sleep(10);
//        }

        cs.schedule(new SubmersibleGrabV2(f, reader).andThen(new SetArmPosition().retract()));

        while (opModeIsActive()) {
            f.update();
        }
    }


    private Command SubGrabWithFailsafe(Follower f, LimelightYoloReader reader){
        return new RepeatUntilCommand(
                ()-> VLRSubsystem.getInstance(ClawSubsystem.class).isSamplePresent(),
                new SubmersibleGrabV2(f, reader),
                new SetArmPosition().retract()
        );
    }
}