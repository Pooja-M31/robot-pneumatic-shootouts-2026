package org.tahomarobotics.robot.util;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

class PneumaticShooterSubsystem extends SubsystemBase {

    private final Compressor compressor;
    private final Solenoid solenoid;

    public PneumaticShooterSubsystem() {
        compressor = new Compressor(PneumaticsModuleType.CTREPCM);
        solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, RobotMap.SHOOTER_SOLENOID_CHANNEL);
        compressor.enableDigital();
    }

    public void fire() {
        solenoid.set(true);
    }

    public void retract() {
        solenoid.set(false);
    }
}


