package org.firstinspires.ftc.teamcode.auto.opModes;

import com.acmerobotics.dashboard.config.Config;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.auto.commands.factory.NetCommandFactory;
import org.firstinspires.ftc.teamcode.helpers.opmode.VLRLinearOpMode;

@Config
@Photon
@Autonomous(name = "RedNetAuto", group = "Red Team")
public class RedNetAuto extends VLRLinearOpMode {

    @Override
    public void run() {
        AutoOpModeRunnner runner = new AutoOpModeRunnner(new NetCommandFactory(false), true);
        runner.initialize(hardwareMap);
        waitForStart();
        runner.run(this::opModeIsActive);
    }

}
