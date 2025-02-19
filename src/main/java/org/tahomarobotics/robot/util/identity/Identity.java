package org.tahomarobotics.robot.util.identity;

public class Identity {
    public static RobotIdentity robotID;

    @SuppressWarnings("ALL")
    public enum RobotIdentity {
        BETA("00:80:2f:32:fd:29"),
        BEEF("00:80:2f:40:6a:02"),
        BEARRACUDA("00:80:2f:33:04:f9");

        private final String addr;

        private RobotIdentity(String addr) {
            this.addr = addr;
        }
    }
}
