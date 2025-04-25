package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.auto.sample.Points_sample.START_POSE;

import com.arcrobotics.ftclib.command.Command;
import com.outoftheboxrobotics.photoncore.Photon;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auto.sample.AutonomousPeriodActionSample;
import org.firstinspires.ftc.teamcode.helpers.opmode.VLRAutoTestOpMode;


@TeleOp(name = "VLR_SampleAuto", group = "!TELEOP")
@Photon
public class VLRSampleAuto extends VLRAutoTestOpMode {
    @Override
    public Pose StartPose() {return START_POSE;}

    @Override
    public Command autoCommand(Follower f){
        return new AutonomousPeriodActionSample(f);
    }
}
