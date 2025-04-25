//package org.firstinspires.ftc.teamcode.helpers.utils.opmodes;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.arcrobotics.ftclib.command.CommandScheduler;
//import com.arcrobotics.ftclib.command.SequentialCommandGroup;
//import com.outoftheboxrobotics.photoncore.Photon;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.util.Constants;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.auto.sample.Points_sample;
//import org.firstinspires.ftc.teamcode.commands.sample.SubmersibleGrab;
//import org.firstinspires.ftc.teamcode.helpers.enums.Alliance;
//import org.firstinspires.ftc.teamcode.helpers.opmode.VLRLinearOpMode;
//import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetSlideExtension;
//import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.limelight.LimelightYoloReader;
//
//import pedroPathing.constants.FConstants;
//import pedroPathing.constants.LConstants;
//
//@TeleOp(name = "LimelightAimer", group = "Utils")
//@Photon
//@Config
//public class LimelightAimerTest extends VLRLinearOpMode {
//
//    public static boolean go = false;
//
//    @Override
//    public void run() {
//        Constants.setConstants(FConstants.class, LConstants.class);
//
//        VLRSubsystem.requireSubsystems(ArmSlideSubsystem.class, ArmRotatorSubsystem.class, ClawSubsystem.class);
//        VLRSubsystem.initializeAll(hardwareMap);
//
//        Follower f = new Follower(hardwareMap);
//        f.setStartingPose(Points_sample.SUB_GRAB_POSE);
//        CommandScheduler cs = CommandScheduler.getInstance();
//        LimelightYoloReader reader = new LimelightYoloReader();
//        VLRSubsystem.getInstance(ClawSubsystem.class).setTargetAngle(0.6);
//
//
//        waitForStart();
//
//        while (!go) {
//            sleep(10);
//        }
//
//        cs.schedule(new SequentialCommandGroup(
//                new SubmersibleGrab(f, Alliance.BLUE, reader),
//                new SetSlideExtension(0)
//        ));
//
//        while (opModeIsActive()) {
//            f.update();
//        }
//    }
//}