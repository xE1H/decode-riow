package org.firstinspires.ftc.teamcode.helpers.persistence;

import org.firstinspires.ftc.teamcode.helpers.enums.Alliance;

import java.util.logging.Logger;

public class AllianceSaver {
    static volatile Alliance alliance;
    static boolean allianceSaved = false;

    public static void setAlliance(Alliance newAlliance) {
        alliance = newAlliance;
        allianceSaved = true;
        Logger.getLogger("AllianceSaver").info("Saved alliance as " + newAlliance);
    }

    public static Alliance getAlliance() {
        if (allianceSaved) return alliance;

        return null;
    }
}
