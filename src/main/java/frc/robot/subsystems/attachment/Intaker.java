package frc.robot.subsystems.attachment;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Intaker extends SubsystemBase {
    public enum IntakerPosition {
        kDown,
        kUp
    }

    public enum IntakerMotorState {
        kReversed,
        kStopped,
        kIntaking
    }

    /**
     * get the intakers position
     * @return the state of the intaker's position, up or down
     */
    public abstract IntakerPosition getIntakerPosition();

    /**
     * get the state of the intaker
     * @return the state of the intaker motors
     */
    public abstract IntakerMotorState getMotorState();

    /**
     * Set the state of the intaker motors
     * @param state the new state of the intaker motors
     */
    public abstract void setMotorState(IntakerMotorState state);

    /**
     * Start intaking
     */
    public void intake() {
        setMotorState(IntakerMotorState.kIntaking);
    }

    /**
     * Stop the intake motors
     */
    public void stop() {
        setMotorState(IntakerMotorState.kStopped);
    }

    /**
     * Start the intaker in reverse
     */
    public void reverse() {
        setMotorState(IntakerMotorState.kReversed);
    }
}
