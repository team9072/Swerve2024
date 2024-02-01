package frc.robot.subsystems.note_handling;

import com.revrobotics.CANSparkLowLevel.MotorType;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;

public class FeederSubsystem extends SubsystemBase {

    public enum FeederState {
        kReversed,
        kStopped,
        kIntaking,
        kShooting();
    }

    private final CANSparkMax m_feederMotor;
    // TODO: replace with propper sensor class
    private boolean m_beamBreakSensor;

    private FeederState m_state = FeederState.kStopped;

    /**
     * Create a new feeder subsystem
     */
    public FeederSubsystem() {
        m_feederMotor = new CANSparkMax(FeederConstants.kFeederMotorCANId, MotorType.kBrushless);

        m_beamBreakSensor = false;
    }

    /**
     * Set the state of the feeder
     * @param state the new state for the feeder
     */
    public void setState(FeederState state) {
        m_state = state;

        double speed = switch(m_state) {
            case kReversed -> FeederConstants.kReverseSpeed;
            case kStopped -> 0;
            case kIntaking -> FeederConstants.kIntakeSpeed;
            case kShooting -> FeederConstants.kShootSpeed;
        };

        m_feederMotor.set(speed);
    }

    /*
     * Get the current state of the feeder
     */
    public FeederState getState() {
        return m_state;
    }

    /**
     * Start the feeder forward
     */
    public void intake() {
        setState(FeederState.kIntaking);
    }

    /**
     * Stop the feeder motors
     */
    public void stop() {
        setState(FeederState.kStopped);
    }

    /**
     * Start the feeder in reverse
     */
    public void reverse() {
        setState(FeederState.kReversed);
    }

    /**
     * Start the feeder forward at a faster speed
     */
    public void startShooting() {
        setState(FeederState.kShooting);
    }
}
