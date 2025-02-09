package org.firstinspires.ftc.teamcode.auto.opModes;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auto.AutoOpModeRunner;
import org.firstinspires.ftc.teamcode.auto.commands.factory.NetCommandFactory;
import org.firstinspires.ftc.teamcode.helpers.autoconfig.AutoConfigurator;
import org.firstinspires.ftc.teamcode.helpers.enums.Alliance;
import org.firstinspires.ftc.teamcode.helpers.opmode.VLRLinearOpMode;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.neopixel.NeoPixelConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.neopixel.NeoPixelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.neopixel.commands.SetBrightness;
import org.firstinspires.ftc.teamcode.subsystems.neopixel.commands.SetColour;
import org.firstinspires.ftc.teamcode.subsystems.neopixel.commands.SetEffect;
import org.firstinspires.ftc.teamcode.subsystems.neopixel.commands.SetEffectTime;

import java.util.concurrent.atomic.AtomicReference;

@Config
@Photon
@Autonomous(name = "NetAuto", group = "!AUTO")
public class NetAuto extends VLRLinearOpMode {

    @Override
    public void run() {
        //VLRSubsystem.initializeOne(hardwareMap, NeoPixelSubsystem.class);

        ElapsedTime opModeTime = new ElapsedTime();
        Alliance alliance;

        AutoConfigurator configurator = new AutoConfigurator(telemetry, gamepad1);

        AutoConfigurator.Choice choice = configurator.multipleChoice("Select alliance:",
                new AutoConfigurator.Choice("Red"),
                new AutoConfigurator.Choice("Blue"));

        alliance = choice.text.equals("Red") ? Alliance.RED : Alliance.BLUE;

        configurator.review("Alliance: " + alliance);
        VLRSubsystem.initializeOne(hardwareMap, NeoPixelSubsystem.class);

        NeoPixelSubsystem np = VLRSubsystem.getInstance(NeoPixelSubsystem.class);

        np.setBrightness(1);
        np.setColor(alliance == Alliance.BLUE ? NeoPixelConfiguration.Colour.BLUE : NeoPixelConfiguration.Colour.RED);
        np.setEffect(NeoPixelConfiguration.Effect.BREATHE);
        np.setEffectTime(3);

        AutoOpModeRunner runner = new AutoOpModeRunner(new NetCommandFactory(alliance, opModeTime));
        runner.initialize(hardwareMap);

        while(!isStarted()) {
            np.periodic();
            try {
                Thread.sleep(30);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }

        }
        np.setEffect(NeoPixelConfiguration.Effect.SOLID_COLOR);
        np.setColor(NeoPixelConfiguration.Colour.YELLOW);
        opModeTime.reset(); // Reset on start for accurate time

        runner.run(this::opModeIsActive, false);
    }
}
