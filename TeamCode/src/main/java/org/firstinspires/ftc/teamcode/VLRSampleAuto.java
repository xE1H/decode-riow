package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.auto.sample.PointsSample.START_POSE;

import com.arcrobotics.ftclib.command.Command;
import com.outoftheboxrobotics.photoncore.Photon;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.sample.AutonomousPeriodActionSample;
import org.firstinspires.ftc.teamcode.helpers.opmode.VLRAutoTestOpMode;
import org.firstinspires.ftc.teamcode.subsystems.limelight.LimelightYoloReader;


@Autonomous(name = "VLR_SampleAuto", group = "!TELEOP")
@Photon
public class VLRSampleAuto extends VLRAutoTestOpMode {
    @Override
    public Pose StartPose() {
        return START_POSE;
    }

    @Override
    public Command autoCommand(Follower f, LimelightYoloReader reader) {
        return new AutonomousPeriodActionSample(f, reader);
    }

    @Override
    public boolean SpecimenOnly(){
        return false;
    }
}
