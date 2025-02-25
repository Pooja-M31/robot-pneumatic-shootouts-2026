package org.tahomarobotics.robot.collector;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.indexer.Indexer;
import org.tahomarobotics.robot.util.RobustConfigurator;
import org.tahomarobotics.robot.util.SubsystemIF;
import org.tahomarobotics.robot.util.game.GamePiece;
import org.tahomarobotics.robot.util.signals.LoggedStatusSignal;
import org.tahomarobotics.robot.util.sysid.SysIdTests;
import org.tinylog.Logger;

import java.util.List;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static org.tahomarobotics.robot.collector.CollectorConstants.*;

@Logged(strategy = Logged.Strategy.OPT_IN)
public class Collector extends SubsystemIF {
    private final static Collector INSTANCE = new Collector();

    // -- Member Variables --

    // Hardware

    private final TalonFX leftMotor; // Leader
    private final TalonFX rightMotor; // Follower
    private final TalonFX collectorMotor;

    // Status Signals

    private final StatusSignal<Angle> leftDeployPosition, rightDeployPosition;
    private final StatusSignal<AngularVelocity> leftDeployVelocity, rightDeployVelocity, collectorVelocity;
    private final StatusSignal<Current> collectorCurrent;

    @Logged
    private final LoggedStatusSignal.List statusSignals;

    // Control Requests

    private final MotionMagicVelocityVoltage velocityControl = new MotionMagicVelocityVoltage(0.0).withEnableFOC(
        RobotConfiguration.RIO_PHOENIX_PRO);
    private final MotionMagicVoltage positionControl = new MotionMagicVoltage(0.0).withEnableFOC(
        RobotConfiguration.RIO_PHOENIX_PRO);

    private final VoltageOut voltageControl = new VoltageOut(0.0);

    // State

    @Logged
    private TargetDeployState targetDeployState = TargetDeployState.ZEROED;
    @Logged
    private TargetCollectorState targetCollectorState = TargetCollectorState.DISABLED;
    @Logged
    private GamePiece collectionMode = GamePiece.CORAL;

    // -- Initialization --

    private Collector() {
        // Create hardware

        leftMotor = new TalonFX(RobotMap.COLLECTOR_LEFT_MOTOR);
        rightMotor = new TalonFX(RobotMap.COLLECTOR_RIGHT_MOTOR);
        collectorMotor = new TalonFX(RobotMap.COLLECTOR_COLLECT_MOTOR);

        // Configure hardware

        RobustConfigurator.tryConfigureTalonFX("Collector Left Motor", leftMotor, deployLeftMotorConfiguration);
        RobustConfigurator.tryConfigureTalonFX(
            "Collector Right Motor", rightMotor,
            deployLeftMotorConfiguration.withMotorOutput(
                deployRightMotorOutputConfiguration)
        );
        RobustConfigurator.tryConfigureTalonFX("Collector Collect Motor", collectorMotor, collectorMotorConfig);

        // Bind status signals

        leftDeployPosition = leftMotor.getPosition();
        rightDeployPosition = rightMotor.getPosition();
        leftDeployVelocity = leftMotor.getVelocity();
        rightDeployVelocity = rightMotor.getVelocity();

        collectorVelocity = collectorMotor.getVelocity();

        StatusSignal<Current> leftDeployCurrent = leftMotor.getSupplyCurrent();
        StatusSignal<Current> rightDeployCurrent = rightMotor.getSupplyCurrent();
        collectorCurrent = collectorMotor.getSupplyCurrent();

        statusSignals = new LoggedStatusSignal.List(List.of(
            new LoggedStatusSignal("Left Deploy Position", leftDeployPosition),
            new LoggedStatusSignal("Right Deploy Position", rightDeployPosition),
            new LoggedStatusSignal("Left Deploy Velocity", leftDeployVelocity),
            new LoggedStatusSignal("Right Deploy Velocity", rightDeployVelocity),
            new LoggedStatusSignal("Collector Velocity", collectorVelocity),
            new LoggedStatusSignal("Left Deploy Current", leftDeployCurrent),
            new LoggedStatusSignal("Right Deploy Current", rightDeployCurrent),
            new LoggedStatusSignal("Collector Collect Current", collectorCurrent)
        ));

        statusSignals.setUpdateFrequencyForAll(RobotConfiguration.MECHANISM_UPDATE_FREQUENCY);
        ParentDevice.optimizeBusUtilizationForAll(leftMotor, rightMotor, collectorMotor);
    }

    public static Collector getInstance() {
        return INSTANCE;
    }

    private static final Indexer indexer = Indexer.getInstance();

    // -- Zeroing --

    public void setZeroingVoltage() {
        leftMotor.setVoltage(DEPLOY_ZEROING_VOLTAGE);
        rightMotor.setVoltage(DEPLOY_ZEROING_VOLTAGE);
    }

    public boolean isDeployStopped() {
        return leftDeployVelocity.getValueAsDouble() < DEPLOY_MOVING_VELOCITY_THRESHOLD &&
               rightDeployVelocity.getValueAsDouble() < DEPLOY_MOVING_VELOCITY_THRESHOLD;
    }

    public void zero() {
        if (RobotState.isDisabled()) { return; }
        Logger.info("Zeroed collector.");

        leftMotor.setPosition(TargetDeployState.ZEROED.angle);
        rightMotor.setPosition(TargetDeployState.ZEROED.angle);

        targetDeployState = TargetDeployState.STOW;
        setDeploymentControl(targetDeployState);
    }

    // -- Deployment State Machine --

    public TargetDeployState getTargetDeployState() {
        return targetDeployState;
    }

    private void setDeploymentControl(TargetDeployState state) {
        leftMotor.setControl(positionControl.withPosition(state.angle));
        rightMotor.setControl(positionControl);
    }

    /**
     * Sync the applied control request for the deployment motors to the target state.
     */
    private void syncDeploymentControl() {
        // Ensure we transition to the correct mode.
        if (isDeploymentCollecting()) {
            targetDeployState = collectionMode == GamePiece.CORAL ?
                TargetDeployState.CORAL_COLLECT :
                TargetDeployState.ALGAE_COLLECT;
        }

        setDeploymentControl(targetDeployState);
    }

    // Transitions

    private void setTargetDeployState(TargetDeployState state) {
        if (checkZeroingGuard()) { return; }

        targetDeployState = state;
        syncDeploymentControl();
    }

    public void deploymentTransitionToStow() {
        setTargetDeployState(TargetDeployState.STOW);

        if (!isHoldingAlgae()) {
            setTargetCollectorState(TargetCollectorState.DISABLED);
        }
    }

    public void deploymentTransitionToCollect() {
        // Sets the target state depending on the collection mode;
        // Is unnecessary due to syncDeploymentControl, but kept for readability.
        setTargetDeployState(
            collectionMode == GamePiece.CORAL ?
                TargetDeployState.CORAL_COLLECT :
                TargetDeployState.ALGAE_COLLECT
        );
    }

    private void deploymentTransitionToEjecting() {
        if (targetDeployState != TargetDeployState.STOW) {
            // Overrides the control request while retaining the previous target state.
            setDeploymentControl(TargetDeployState.EJECT);
        }
    }

    private void deploymentCancelEjecting() {
        syncDeploymentControl();
    }

    // Checks

    public boolean isDeploymentStowed() {
        return targetDeployState == TargetDeployState.STOW;
    }

    public boolean isDeploymentCollecting() {
        return targetDeployState == TargetDeployState.CORAL_COLLECT || targetDeployState == TargetDeployState.ALGAE_COLLECT;
    }

    public boolean isHoldingAlgae() {
        return targetCollectorState == TargetCollectorState.HOLDING_ALGAE;
    }

    // -- Collector State Machine --

    public TargetCollectorState getTargetCollectorState() {
        return targetCollectorState;
    }

    private void collectorStateMachine() {
        switch (targetCollectorState) {
            case COLLECTING -> {
                if (collectorCurrent.getValueAsDouble() > ALGAE_HOLDING_CURRENT_THRESHOLD && collectionMode == GamePiece.ALGAE) {
                    collectorTransitionToHolding();
                }
            }
        }
    }

    private void setTargetCollectorState(TargetCollectorState state) {
        if (checkZeroingGuard()) { return; }

        targetCollectorState = state;
        switch (state.type) {
            case VELOCITY -> collectorMotor.setControl(velocityControl.withVelocity(state.value));
            case VOLTAGE -> collectorMotor.setControl(voltageControl.withOutput(state.value));
            case NONE -> collectorMotor.stopMotor();
        }
    }

    // Transitions

    public void collectorTransitionToDisabled() {
        setTargetCollectorState(TargetCollectorState.DISABLED);
    }

    public void collectorTransitionToHolding() {
        setTargetCollectorState(TargetCollectorState.HOLDING_ALGAE);
    }

    public void collectorTransitionToCollecting() {
        setTargetCollectorState(TargetCollectorState.COLLECTING);
    }

    private void collectorTransitionToEjecting() {
        setTargetCollectorState(TargetCollectorState.EJECTING);
    }

    private void collectorCancelEjecting() {
        setTargetCollectorState(TargetCollectorState.DISABLED);
    }

    // -- Combined Transitions --

    public void transitionToEjecting() {
        deploymentTransitionToEjecting();
        collectorTransitionToEjecting();
    }

    public void cancelEjecting() {
        deploymentCancelEjecting();
        collectorCancelEjecting();
    }

    // -- Subsystem Overrides --


    @Override
    public SubsystemIF initialize() {
        new Trigger(() -> RobotState.isEnabled() && !RobotState.isAutonomous())
            .onTrue(
                CollectorCommands
                    .createZeroCommand(this)
                    .onlyIf(() -> targetDeployState == TargetDeployState.ZEROED)
            );

        return this;
    }

    @Override
    public void periodic() {
        statusSignals.refreshAll();
        collectorStateMachine();
    }

    // -- Collection Mode --

    public GamePiece getCollectionMode() {
        return collectionMode;
    }

    public GamePiece toggleCollectionMode() {
        collectionMode = collectionMode == GamePiece.CORAL ? GamePiece.ALGAE : GamePiece.CORAL;
        syncDeploymentControl();

        return collectionMode;
    }

    public void setCollectionMode(GamePiece collectionMode) {
        this.collectionMode = collectionMode;
        syncDeploymentControl();
    }

    // -- Helper Methods --

    private boolean checkZeroingGuard() {
        if (targetDeployState == TargetDeployState.ZEROED) {
            Logger.error("Attempted to use collector prior to zeroing.");
            return true;
        }
        return false;
    }

    // -- SysId --


    @Override
    public List<SysIdTests.Test> getSysIdTests() {
        return List.of(
            SysIdTests.characterize(
                "Collector Deployment",
                this,
                leftMotor,
                Volts.of(0.125).per(Second),
                Volts.of(0.5),
                rightMotor,
                true
            ),
            SysIdTests.characterize(
                "Collector Collector",
                this,
                collectorMotor,
                Volts.of(1).per(Second),
                Volts.of(3)
            )
        );
    }
}