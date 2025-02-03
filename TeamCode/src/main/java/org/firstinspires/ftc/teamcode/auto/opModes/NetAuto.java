package org.firstinspires.ftc.teamcode.auto.opModes;

import com.acmerobotics.dashboard.config.Config;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auto.AutoOpModeRunner;
import org.firstinspires.ftc.teamcode.auto.commands.factory.NetCommandFactory;
import org.firstinspires.ftc.teamcode.helpers.autoconfig.AutoConfigurator;
import org.firstinspires.ftc.teamcode.helpers.enums.Alliance;
import org.firstinspires.ftc.teamcode.helpers.opmode.VLRLinearOpMode;

import java.util.concurrent.atomic.AtomicReference;

@Config
@Photon
@Autonomous(name = "NetAuto", group = "!AUTO")
public class NetAuto extends VLRLinearOpMode {

    @Override
    public void run() {
        AutoConfigurator configurator = new AutoConfigurator(telemetry, gamepad1);
        Alliance alliance;

        AutoConfigurator.Choice choice = configurator.multipleChoice("Select alliance:",
                new AutoConfigurator.Choice("Red"),
                new AutoConfigurator.Choice("Blue"));

        alliance = choice.text.equals("Red") ? Alliance.RED : Alliance.BLUE;


        configurator.review("Alliance: " + alliance);

        ElapsedTime time = new ElapsedTime();

        AutoOpModeRunner runner = new AutoOpModeRunner(new NetCommandFactory(alliance, time));
        runner.initialize(hardwareMap);

        waitForStart();

        time.reset();
        runner.run(this::opModeIsActive, false);
    }
}
