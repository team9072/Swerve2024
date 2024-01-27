package frc.robot.subsystems.feeder;

import com.revrobotics.CANSparkLowLevel.MotorType;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.UTBIntakerConstants;
import frc.robot.subsystems.intakers.UTBIntakerSubsystem;

public class FeederSubsystem extends SubsystemBase {

    public enum FeederState {
        // can go to INTAKE
        STOPPED(0, 0, 1),
        // can go to IDLE and NOTE_LOADED
        INTAKE(1, FeederConstants.kFeederIntakeSpeed, 0, 2),
        // can go to SHOOTING
        NOTE_LOADED(2, 0, 3),
        // can go to IDLE
        SHOOTING(3, FeederConstants.kFeederShootSpeed, 0);

        public final int m_Id;
        public final double m_feederSpeed;
        public final int[] m_transitons;

        FeederState(int id, double feederSpeed, int... transitions) {
            m_Id = id;
            m_feederSpeed = feederSpeed;
            m_transitons = transitions;
        }

        public boolean canTransitionTo(FeederState newState) {
            if (this == newState) {
                return true;
            }
            ;
            for (int id : m_transitons) {
                if (id == newState.m_Id) {
                    return true;
                }
            }

            return false;
        }
    }

    private final CANSparkMax m_feederMotor1;
    private final CANSparkMax m_feederMotor2;
    // TODO: replace with propper sensor class
    private boolean m_beamBreakSensor;

    private FeederState m_state = FeederState.STOPPED;
    private IFeederListener[] m_registeredListners;

    /**
     * Create a new feeder subsystem
     */
    public FeederSubsystem(IFeederListener... listners) {
        m_feederMotor1 = new CANSparkMax(UTBIntakerConstants.kIntakeMotor1CANId, MotorType.kBrushless);
        m_feederMotor2 = new CANSparkMax(UTBIntakerConstants.kIntakeMotor2CANId, MotorType.kBrushless);

        m_beamBreakSensor = false;

        for (IFeederListener listner : listners) {
            listner.onFeederRegister(this);
        }
    }

    private void forceSetState(FeederState state) {
        m_state = state;
        m_feederMotor1.set(m_state.m_feederSpeed);
        m_feederMotor2.set(m_state.m_feederSpeed);

        for (IFeederListener listneer : m_registeredListners) {
            listneer.handleFeederChange(m_state, this);
        }
    }

    private boolean trySetState(FeederState state) {
        if (m_state.canTransitionTo(FeederState.INTAKE)) {
            forceSetState(state);
            return true;
        }

        return false;
    }

    @Override
    public void periodic() {
        // TODO: get beam break state for real
        boolean beamBreakState = m_beamBreakSensor;

        if (m_state == FeederState.INTAKE && beamBreakState) {
            // turn off feeder when get note
            trySetState(FeederState.NOTE_LOADED);
        }

    }

    /*
     * Get the current state of the feeder
     */
    public FeederState getState() {
        return m_state;
    }

    /**
     * Check if the feeder can swith to intaking
     * @return true if the feeder can start intaking
     */
    public boolean canStartIntaking() {
        return m_state.canTransitionTo(FeederState.INTAKE);
    }

    /**
     * Check if the feeder can swith to shooting
     * @return true if the feeder can start shooting
     */
    public boolean canStartShooting() {
        return m_state.canTransitionTo(FeederState.SHOOTING);
    }

    /**
     * Check if the feeder can switch to stopped
     * Note that this is different from NOTE_LOADED, which also stops it.
     * @return true if the feeder can switch to stopped
     */
    public boolean canStopFeeder() {
        return m_state.canTransitionTo(FeederState.STOPPED);
    }

    /**
     * Check if the feeder has a note loaded
     * @return true if a note is loaded
     */
    public boolean isNoteLoaded() {
        return m_state == FeederState.NOTE_LOADED;
    }

    /**
     * Try switching the feeder to intaking
     * It may or may not switch the feeder, so make sure it can switch to the
     * desired state by calling {@link #canStartIntaking() canStartintaking}
     * @return true if it was able to switch, false otherwise
     */
    public boolean tryStartIntaking() {
        return trySetState(FeederState.INTAKE);
    }

    /**
     * Try switching the feeder to shooting
     * It may or may not switch the feeder, so make sure it can switch to the
     * desired state by calling {@link #canStartShooting() canStartShooting}
     * @return true if it was able to switch, false otherwise
     */
    public boolean tryStartShooting() {
        return trySetState(FeederState.SHOOTING);
    }

    /**
     * Try switching the feeder to stopped
     * It may or may not switch the feeder, so make sure it can switch to the
     * desired state by calling {@link #canStopFeeder() canStopFeeder}
     * Note that this is different from NOTE_LOADED, which also stops it.
     * @return true if it was able to switch, false otherwise
     */
    public boolean tryStopFeeder() {
        return trySetState(FeederState.STOPPED);
    }

    /**
     * Get a command to try switching the feeder to intaking
     * It may or may not switch the feeder, so make sure itcan switch to the
     * desired state by calling {@link #canStartIntaking() canStartintaking}
     * @return A command to set the feeder state
     */
    public Command getstartIntakingCommand() {
        return runOnce(() -> tryStartIntaking());
    }

    /**
     * Get a command to try switching the feeder to intaking
     * It may or may not switch the feeder, so make sure it can switch to the
     * desired state by calling {@link #canStartShooting() canStartShooting}
     * @return A command to set the feeder state
     */
    public Command getStartShootingCommand() {
        return runOnce(() -> tryStartShooting());
    }

    /**
     * Get a command to try switching the feeder to stopped
     * It may or may not switch the feeder, so make sure it can switch to the
     * desired state by calling {@link #canStopFeeder() canStopFeeder}
     * Note that this is different from NOTE_LOADED, which also stops it.
     * @return A command to set the feeder state
     */
    public Command getStopFeederCommand() {
        return runOnce(() -> tryStopFeeder());
    }

    /**
     * Get a command to start shooting for a number of seconds
     * It may or may not switch the feeder, so make sure it can switch to the
     * desired state by calling {@link #canStartShooting() canStopFeeder}
     * @return A command to shoot for a number of seconds
     */
    public Command getShootCommand(double seconds) {
        return new SequentialCommandGroup(
                getStartShootingCommand(),
                new WaitCommand(seconds),
                getStopFeederCommand()).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }
}
