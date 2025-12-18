package org.tahomarobotics.robot.util;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class PneumaticShootouts {

    private final DoubleSolenoid leftShootout;
    private final DoubleSolenoid rightShootout;
    private boolean deployed = false;

    public PneumaticShootouts(DoubleSolenoid leftShootout, DoubleSolenoid rightShootout) {
        this.leftShootout = leftShootout;
        this.rightShootout = rightShootout;
    }

    public PneumaticShootouts() {
        leftShootout = new DoubleSolenoid(
            1,
            PneumaticsModuleType.CTREPCM,
            RobotMap.LEFT_FORWARD,
            RobotMap.LEFT_REVERSE
        );

        rightShootout = new DoubleSolenoid(
            1,
            PneumaticsModuleType.CTREPCM,
            RobotMap.RIGHT_FORWARD,
            RobotMap.RIGHT_REVERSE
        );
    }

    public void shoot() {
        leftShootout.set(Value.kForward);
        rightShootout.set(Value.kForward);
        deployed = true;
    }

    public void retract() {
        leftShootout.set(Value.kReverse);
        rightShootout.set(Value.kReverse);
        deployed = false;
    }

    public void off() {
        leftShootout.set(Value.kOff);
        rightShootout.set(Value.kOff);
    }

    public boolean isDeployed() {
        return deployed;
    }

    private void setDeployed(boolean deployed) {
        this.deployed = deployed;
    }
}
