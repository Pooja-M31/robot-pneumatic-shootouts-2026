package org.tahomarobotics.robot.climber;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.RobustConfigurator;
import org.tahomarobotics.robot.util.SubsystemIF;
import org.tahomarobotics.robot.util.signals.LoggedStatusSignal;
import org.tahomarobotics.robot.util.sysid.SysIdTests;
import org.tinylog.Logger;

import java.util.List;

import static edu.wpi.first.units.Units.*;
import static org.tahomarobotics.robot.climber.ClimberConstants.*;

@Logged(strategy = Logged.Strategy.OPT_IN)
public class Climber extends SubsystemIF {
    private final static Climber INSTANCE = new Climber();

    // -- Member Variables --

    // Hardware

    private final TalonFX climberMotor = new TalonFX(RobotMap.CLIMBER_MOTOR);
    private final TalonFX climberFollower = new TalonFX(RobotMap.CLIMBER_FOLLOWER);
    private final VictorSPX ratchetSolenoid = new VictorSPX(RobotMap.CLIMBER_RATCHET_SOLENOID);

    // Status Signals

    @Logged
    private final LoggedStatusSignal.List statusSignals;
    private final StatusSignal<Current> climberMotorCurrent;
    private final StatusSignal<Current> climberFollowerCurrent;
    private final StatusSignal<Angle> climberMotorPosition;
    private final StatusSignal<Angle> climberFollowerPosition;
    private final StatusSignal<Voltage> climberMotorVoltage;
    private final StatusSignal<Voltage> climberFollowerVoltage;

    // Control Requests

    private final MotionMagicVoltage positionControl = new MotionMagicVoltage(0);

    // State

    @Logged
    private ClimberState climbState = ClimberState.STOWED;
    @Logged
    private double targetPosition;
    @Logged
    private boolean isZeroed;
    @Logged
    private double solenoidVoltage;

    // -- Initialization --

    private Climber() {
        RobustConfigurator.tryConfigureTalonFX("Climber Motor", climberMotor, climberMotorConfig);
        climberFollower.setControl(new Follower(RobotMap.CLIMBER_MOTOR, true));

        climberMotorCurrent = climberMotor.getSupplyCurrent();
        climberFollowerCurrent = climberFollower.getSupplyCurrent();
        climberMotorPosition = climberMotor.getPosition();
        climberFollowerPosition = climberFollower.getPosition();
        climberMotorVoltage = climberMotor.getMotorVoltage();
        climberFollowerVoltage = climberFollower.getMotorVoltage();

        statusSignals = new LoggedStatusSignal.List(List.of(
            new LoggedStatusSignal("Climber Motor Position", climberMotorPosition),
            new LoggedStatusSignal("Climber Follower Position", climberFollowerPosition),
            new LoggedStatusSignal("Climber Motor Current", climberMotorCurrent),
            new LoggedStatusSignal("Climber Follower Current", climberFollowerCurrent),
            new LoggedStatusSignal("Climber Motor Voltage", climberMotorVoltage),
            new LoggedStatusSignal("Climber Follower Voltage", climberFollowerVoltage)
        ));

        statusSignals.setUpdateFrequencyForAll(RobotConfiguration.MECHANISM_UPDATE_FREQUENCY);
        ParentDevice.optimizeBusUtilizationForAll(climberMotor, climberFollower);
    }

    public static Climber getInstance() {
        return INSTANCE;
    }

    @Override
    public Climber initialize() {
        return this;
    }

    @Override
    public void onTeleopInit() {
        if (!isZeroed) {
            zeroPosition();
            isZeroed = true;
        }
        stow();
    }

    @Override
    public void onAutonomousInit() {
        if (!isZeroed) {
            zeroPosition();
            isZeroed = true;
        }
        stow();
    }

    // -- Getters --

    @Logged
    @Override
    public double getTotalCurrent() {
        return climberMotorCurrent.getValueAsDouble() + climberFollowerCurrent.getValueAsDouble();
    }

    public ClimberState getClimbState() {
        return climbState;
    }

    @Logged
    public boolean isAtTargetPosition() {
        return (Math.abs(climberMotorPosition.getValueAsDouble() - targetPosition) < ClimberConstants.CLIMB_POSITION_TOLERANCE);
    }

    // -- Climb Control --

    public void stow() {
        targetPosition = ClimberConstants.STOW_POSITION;
        Logger.info("Climber stow");
        climberMotor.setControl(positionControl.withPosition(targetPosition).withSlot(0));

        climbState = ClimberState.STOWED;
    }

    public void deploy() {
        targetPosition = ClimberConstants.DEPLOY_POSITION;
        Logger.info("Climber deploy");
        climberMotor.setControl(positionControl.withPosition(targetPosition).withSlot(0));

        climbState = ClimberState.DEPLOYED;
    }

    public void climb() {
        targetPosition = ClimberConstants.CLIMB_POSITION;
        Logger.info("Climber climb");
        climberMotor.setControl(positionControl.withPosition(targetPosition).withSlot(1));

        climbState = ClimberState.CLIMBED;
    }

    public void deploySolenoid() {
        ratchetSolenoid.set(VictorSPXControlMode.PercentOutput, ClimberConstants.RATCHET_SOLENOID_DEPLOY_PERCENTAGE);
    }

    public void disableSolenoid() {
        Logger.info("Disable solenoid");
        ratchetSolenoid.set(VictorSPXControlMode.PercentOutput, 0);
    }

    public void disableClimberMotors() {
        climberMotor.stopMotor();
    }

    public void zeroPosition() {
        climberMotor.setPosition(ClimberConstants.CLIMBER_ZERO_POSITION);
        climberFollower.setPosition(ClimberConstants.CLIMBER_ZERO_POSITION);
    }

    // -- Periodic --

    @Override
    public void periodic() {
        statusSignals.refreshAll();
        solenoidVoltage = ratchetSolenoid.getMotorOutputVoltage();
    }

    // -- States --

    public enum ClimberState {
        STOWED,
        DEPLOYED,
        CLIMBED
    }

    // -- SysId --

    @Override
    public List<SysIdTests.Test> getSysIdTests() {
        return List.of(SysIdTests.characterize(
            "Climber",
            this,
            climberMotor,
            Volts.of(0.25).per(Second),
            Volts.of(1),
            climberFollower,
            true));
    }
}