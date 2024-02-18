package frc.robot.subsystems.attachment;

import edu.wpi.first.wpilibj2.command.Command;
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
     * Set the intaker position, up or down
     * @param position the new position
     */
    public abstract void setIntakerPosition(IntakerPosition position);

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
     * Get a command to set the motor state
     * @param state the new motor state
     * @return A command that sets the state to the provided value
     */
    public Command getSetMotorStateCommand(IntakerMotorState state) {
        return runOnce(() -> setMotorState(state));
    }
}
