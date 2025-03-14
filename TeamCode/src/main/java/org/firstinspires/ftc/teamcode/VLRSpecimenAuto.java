package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Points_specimen.START_POSE;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.outoftheboxrobotics.photoncore.Photon;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.commands.specimen.AutonomousPeriodActions;
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

    /**
     * @noinspection unchecked
     */
    @Override
    public void run() {
        Constants.setConstants(FConstants.class, LConstants.class);

        cs = CommandScheduler.getInstance();

        VLRSubsystem.requireSubsystems(ArmSlideSubsystem.class, ArmRotatorSubsystem.class, ClawSubsystem.class);
        VLRSubsystem.initializeAll(hardwareMap);

        f = new Follower(hardwareMap);
        f.setStartingPose(START_POSE);

        cs.schedule(new AutonomousPeriodActions(f));


        while (opModeIsActive()) {
            f.update();
        }
    }
}
