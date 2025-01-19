package org.firstinspires.ftc.teamcode.auto.opModes;

import com.acmerobotics.dashboard.config.Config;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.auto.commands.factory.ObservationCommandFactory;
import org.firstinspires.ftc.teamcode.helpers.opmode.VLRLinearOpMode;

@Config
@Photon
@Autonomous(name = "RedObservationAuto", group = "Red Team")
public class RedObservationAuto extends VLRLinearOpMode {

    @Override
    public void run() {
        AutoOpModeRunnner runner = new AutoOpModeRunnner(new ObservationCommandFactory(false), false);
        runner.initialize(hardwareMap);
        waitForStart();
        runner.run(this::opModeIsActive, false);
    }
}
