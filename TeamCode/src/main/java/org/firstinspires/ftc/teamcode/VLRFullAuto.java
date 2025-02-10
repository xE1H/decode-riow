package org.firstinspires.ftc.teamcode;

import com.outoftheboxrobotics.photoncore.Photon;
import com.pedropathing.commands.FollowPath;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.helpers.opmode.VLRLinearOpMode;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.hang.HangSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.vision.Vision;

@TeleOp(name = "VLRFullAuto", group = "!TELEOP")
@Photon
public class VLRFullAuto extends VLRLinearOpMode {
    Follower follower;
    Pose startPose = new Pose(10, 111.5, 0);

    /** @noinspection unchecked*/
    @Override
    public void run() {
        VLRSubsystem.requireSubsystems(ArmSlideSubsystem.class, ArmRotatorSubsystem.class, ClawSubsystem.class, HangSubsystem.class, Vision.class);
        VLRSubsystem.initializeAll(hardwareMap);

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);


    }
}
