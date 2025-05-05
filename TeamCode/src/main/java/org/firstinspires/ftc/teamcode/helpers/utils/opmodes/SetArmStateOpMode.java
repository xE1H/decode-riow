package org.firstinspires.ftc.teamcode.helpers.utils.opmodes;

import static java.lang.Thread.sleep;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.helpers.opmode.VLRLinearOpMode;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmState;

@TeleOp
public class SetArmStateOpMode extends VLRLinearOpMode {
    @Override
    public void run(){
        ArmState.set(ArmState.State.SAMPLE_SCORE);

        waitForStart();
        if (opModeIsActive()) {
            ArmState.set(ArmState.State.IN_ROBOT);
        }
    }
}
