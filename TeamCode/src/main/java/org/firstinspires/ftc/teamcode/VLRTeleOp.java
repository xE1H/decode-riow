package org.firstinspires.ftc.teamcode;

import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.controls.PrimaryDriverTeleOpControls;
import org.firstinspires.ftc.teamcode.controls.SecondaryDriverTeleOpControls;
import org.firstinspires.ftc.teamcode.helpers.opmode.VLRLinearOpMode;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.helpers.utils.GlobalConfig;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmState;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.chassis.Chassis;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.hang.HangSubsystem;


/**
 * @noinspection unchecked
 */
@Photon
@TeleOp(name = "VLRTeleOp", group = "!TELEOP")
public class VLRTeleOp extends VLRLinearOpMode {
    // Controls
    PrimaryDriverTeleOpControls primaryDriver;
    SecondaryDriverTeleOpControls secondaryDriver;

    @Override
    public void run() {
        VLRSubsystem.requireSubsystems(Chassis.class, ArmSlideSubsystem.class, ArmRotatorSubsystem.class);
        VLRSubsystem.initializeAll(hardwareMap);

//        VLRSubsystem.getInstance(Chassis.class).enableFieldCentric();
        ArmSlideSubsystem ass = VLRSubsystem.getInstance(ArmSlideSubsystem.class);
        primaryDriver = new PrimaryDriverTeleOpControls(gamepad1);

        waitForStart();
        // since judges are pizdabolai
        VLRSubsystem.initializeOne(hardwareMap, ClawSubsystem.class); // Modified the claw subsystem
        // a bit, now it just *might* work like this now. Removed everything that is not used
        // (like analog feedback lines) and removed empty periodic function.
        VLRSubsystem.initializeOne(hardwareMap, HangSubsystem.class);
        secondaryDriver = new SecondaryDriverTeleOpControls(gamepad2);

        ass.setMotorPower(-0.6);
        ElapsedTime timeout = new ElapsedTime();
        while (!ass.getLimitSwitchState()) {
            sleep(10);
            if (timeout.milliseconds() > 1000) {
                break;
            }
        }
        ass.setMotorPower(0);
        ass.checkLimitSwitch();

        while (opModeIsActive()) {
            primaryDriver.update();
            secondaryDriver.update();
        }
    }
}
