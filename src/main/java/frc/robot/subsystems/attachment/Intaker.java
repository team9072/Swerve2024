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
     * Raise the intaker so it is inside the frame
     */
    public void raiseIntaker() {
        setIntakerPosition(IntakerPosition.kUp);
    }

    /**
     * Raise the intaker so it is inside the frame
     * @return A command to raise the intaker
     */
    public Command getRaiseIntakerCommand() {
        return this.runOnce(this::raiseIntaker);
    }

    /**
     * Drop the intaker so it can intake notes
     */
    public void dropIntaker() {
        setIntakerPosition(IntakerPosition.kDown);
    }

    /**
     * Drop the intaker so it can intake notes
     * @return A command to drop the intaker
     */
    public Command getDropIntakerCommand() {
        return this.runOnce(this::dropIntaker);
    }

    /**
     * Start intaking
     */
    public void intake() {
        setMotorState(IntakerMotorState.kIntaking);
    }

    /**
     * Start intaking
     * @return A command to start intaking
     */
    public Command getIntakeCommand() {
        return this.runOnce(this::intake);
    }

    /**
     * Stop the intake motors
     */
    public void stop() {
        setMotorState(IntakerMotorState.kStopped);
    }

    /**
     * Stop the intake motors
     * @return A command to stop the intake motors
     */
    public Command getStopCommand() {
        return this.runOnce(this::stop);
    }

    /**
     * Start the intaker in reverse
     */
    public void reverse() {
        setMotorState(IntakerMotorState.kReversed);
    }

    /**
     * Start the intaker in reverse
     * @return A command to start the intaker in reverse
     */
    public Command getReverseCommand() {
        return this.runOnce(this::reverse);
    }
}
