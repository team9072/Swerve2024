package frc.robot.subsystems.attachment;

import com.revrobotics.CANSparkLowLevel.MotorType;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
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
    private final boolean m_beamBreakSensor;

    private FeederState m_state = FeederState.kStopped;

    /**
     * Create a new feeder subsystem
     */
    public FeederSubsystem() {
        m_feederMotor = new CANSparkMax(FeederConstants.kFeederMotorCANId, MotorType.kBrushless);

        m_feederMotor.restoreFactoryDefaults();
        m_feederMotor.setIdleMode(IdleMode.kCoast);

        m_beamBreakSensor = false;
    }

    /**
     * Get the state of the beam break sensor
     * @return true if a note is detected, or fale otherwise
     */
    public boolean getBeamBreakState() {
        return m_beamBreakSensor;
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

    /**
     * set the state of the feeder
     * @param state the new state to set
     * @return a command to set the state of the feeder
     */
    public Command getSetStateCommand(FeederState state) {
        return this.runOnce(() -> setState(state));
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
     * Start the feeder forward
     * @return A command to start the feeder forward
     */
    public Command getIntakeCommand() {
        return this.runOnce(this::intake);
    }

    /**
     * Stop the feeder motors
     */
    public void stop() {
        setState(FeederState.kStopped);
    }

    /**
     * Stop the feeder motors
     * @return A command to stop the feeder motors
     */
    public Command getStopCommand() {
        return this.runOnce(this::stop);
    }

    /**
     * Start the feeder in reverse
     */
    public void reverse() {
        setState(FeederState.kReversed);
    }

    /**
     * Start the feeder in reverse
     * @return A command to start the feeder in reverse
     */
    public Command getReverseCommand() {
        return this.runOnce(this::reverse);
    }

    /**
     * Start the feeder forward at shooting speed
     */
    public void startShooting() {
        setState(FeederState.kShooting);
    }

    /**
     * Start the feeder forward at shooting speed
     * @return A command to start the feeder forward at shooting speed
     */
    public Command getstartShootingCommand() {
        return this.runOnce(this::startShooting);
    }
}
