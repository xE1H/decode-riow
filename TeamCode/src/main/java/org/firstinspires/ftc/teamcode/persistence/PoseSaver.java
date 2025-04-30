package org.firstinspires.ftc.teamcode.persistence;

import com.pedropathing.localization.Pose;

public final class PoseSaver {
    static volatile Pose pedroPose = new Pose(0, 0, 0);
    static boolean poseSaved = false;

    public static void setPedroPose(Pose pose) {
        pedroPose = pose;
        poseSaved = true;
    }

    public static Pose getPedroPose() {
        return pedroPose;
    }

    public static boolean isPoseSaved() {
        return poseSaved;
    }
}
