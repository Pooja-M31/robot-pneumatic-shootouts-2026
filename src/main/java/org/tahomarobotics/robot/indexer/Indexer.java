package org.tahomarobotics.robot.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.RobustConfigurator;
import org.tahomarobotics.robot.util.SubsystemIF;
import org.tahomarobotics.robot.util.sysid.SysIdTests;

import java.util.List;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static org.tahomarobotics.robot.indexer.IndexerConstants.*;

public class Indexer extends SubsystemIF {
    private static final Indexer INSTANCE = new Indexer();

    // -- Member Variables --

    // Hardware

    private final TalonFX motor;

    private final DigitalInput beambreak;

    // Status Signals

    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Current> current;

    // Control Requests

    private final MotionMagicVelocityVoltage velocityControl = new MotionMagicVelocityVoltage(0);
    private final MotionMagicVoltage positionControl = new MotionMagicVoltage(0).withSlot(1);

    // State

    @Logged
    private IndexerState state = IndexerState.DISABLED;

    // -- Initialization --

    private Indexer() {
        // Create hardware

        motor = new TalonFX(RobotMap.INDEXER_MOTOR);

        beambreak = new DigitalInput(RobotMap.BEAM_BREAK);

        // Configure hardware

        RobustConfigurator.tryConfigureTalonFX("Indexer Motor", motor, configuration);

        // Bind status signals

        position = motor.getPosition();
        velocity = motor.getVelocity();
        current = motor.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
            RobotConfiguration.MECHANISM_UPDATE_FREQUENCY, position, velocity, current);
        ParentDevice.optimizeBusUtilizationForAll(motor);

        SmartDashboard.putBoolean("arm at position", true);
    }

    public static Indexer getInstance() {
        return INSTANCE;
    }

    // -- State Machine --

    private void setTargetState(IndexerState state) {
        this.state = state;

        switch (state.type) {
            case VELOCITY -> motor.setControl(velocityControl.withVelocity(state.value));
            case NONE -> motor.stopMotor();
        }
    }

    private void stateMachine() {
        switch (state) {
            case COLLECTING -> {
//                if (beamBreakTripped() && isArmAtPassing()) transitionToPassing();
//                if (beamBreakTripped() && !isArmAtPassing()) transitionToHolding();
            }
            case PASSING -> {
                if (!beamBreakTripped()) {
                    transitionToCollected();
                }
            }
            case HOLDING -> {
                if (isArmAtPassing()) {
                    transitionToPassing();
                }
            }
            default -> {}
        }
    }

    // Transitions

    public void transitionToDisabled() {
        if (state == IndexerState.COLLECTING || state == IndexerState.EJECTING) {
            setTargetState(IndexerState.DISABLED);
        }
    }

    public void transitionToCollected() {
        setTargetState(IndexerState.COLLECTED);
    }

    public void transitionToCollecting() {
        if (state != IndexerState.DISABLED) return;
        setTargetState(IndexerState.COLLECTING);
    }

    public void transitionToHolding() {
        setTargetState(IndexerState.HOLDING);
    }

    private void transitionToPassing() {
        setTargetState(IndexerState.PASSING);
    }

    public void transitionToEjecting() {
        setTargetState(IndexerState.EJECTING);
    }

    // -- Getter(s) --

    @Logged
    public double getPosition() {
        return position.getValueAsDouble();
    }

    @Logged
    public double getVelocity() {
        return velocity.getValueAsDouble();
    }

    @Logged
    public double getSupplyCurrent() {
        return current.getValueAsDouble();
    }

    // -- Triggers --
    @Logged
    public boolean beamBreakTripped() {
        return !beambreak.get();
    }

    @Logged
    public boolean isCollecting() {
        return state == IndexerState.COLLECTING;
    }

    @Logged
    public boolean isEjecting() {
        return state == IndexerState.EJECTING;
    }

    @Logged
    public boolean isPassing() {
        return state == IndexerState.PASSING;
    }

    @Logged
    public boolean isCollected() {
        return state == IndexerState.COLLECTED;
    }

    @Logged
    public boolean isHolding() {
        return state == IndexerState.HOLDING;
    }

    @Logged
    public boolean isDisabled() {
        return state == IndexerState.DISABLED;
    }

    @Logged
    public boolean isArmAtPassing() {
        // TODO: Check if the arm is in the collected position
        return SmartDashboard.getBoolean("arm at position", true);
    }

    // -- Setter(s) --

    private void zeroPosition() {
        motor.setPosition(0);
    }

    // -- Periodic --

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(position, velocity, current);
        stateMachine();
    }

    // -- SysId --

    @Override
    public List<SysIdTests.Test> getSysIdTests() {
        return List.of(
            SysIdTests.characterize(
                "Indexer SysId Test",
                this,
                motor,
                Volts.of(1).per(Second),
                Volts.of(3)
            ));
    }
}


