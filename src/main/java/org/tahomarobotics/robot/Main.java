package org.tahomarobotics.robot;

import edu.wpi.first.wpilibj.RobotBase;
import org.tahomarobotics.robot.util.identity.Identity;
import org.tinylog.Logger;

public final class Main {
    private Main() {}

    public static void main(String... args) {
        int idx = 0;
        if (args.length == 0) {
            Logger.warn("No identity index passed in! Defaulting to {}", Identity.RobotIdentity.values()[0]);
        } else {
            try {
                idx = Integer.parseInt(args[0]);
            } catch (NumberFormatException e) {
                Logger.warn("Invalid identity index passed in! Defaulting to {}", Identity.RobotIdentity.values()[0]);
            }
        }

        Identity.robotID = Identity.RobotIdentity.values()[idx];
        Logger.info("Identity: {}", Identity.robotID);

        RobotBase.startRobot(Robot::new);
    }
}
