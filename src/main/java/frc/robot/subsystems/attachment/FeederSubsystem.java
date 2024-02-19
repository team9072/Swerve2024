package frc.robot.subsystems.attachment;

import com.revrobotics.CANSparkLowLevel.MotorType;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;

public class FeederSubsystem extends SubsystemBase {

    public enum FeederState {
        kReversed,
        kStopped,
        kIntaking,
        kShooting;
    }

    public enum FeederPivotPosition {
        kIntakePosition(FeederConstants.PivotLimits.kIntakeMin, FeederConstants.PivotLimits.kIntakeMax),
        kSpeakerPosition(FeederConstants.PivotLimits.kSpeakerMin, FeederConstants.PivotLimits.kSpeakerMax),
        kAmpPosition(FeederConstants.PivotLimits.kAmpPos, FeederConstants.PivotLimits.kAmpPos);

        public double lowLimit, highLimit;

        FeederPivotPosition(double low, double high) {
            lowLimit = Math.max(FeederConstants.PivotLimits.kGlobalMin, low);
            highLimit = Math.min(FeederConstants.PivotLimits.kGlobalMax, high);
        }
    }

    private final CANSparkMax m_feederMotor;
    private final CANSparkMax m_pivotMotor;
    private SparkPIDController m_pivotPID;
    private RelativeEncoder m_pivotEncoder;

    private final DigitalInput m_beamBreakSensor;

    private FeederState m_state = FeederState.kStopped;
    private FeederPivotPosition m_pivotPosition = FeederPivotPosition.kIntakePosition;
    private double m_pivotSetpoint = 0;

    /**
     * Create a new feeder subsystem
     */
    public FeederSubsystem() {
        m_feederMotor = new CANSparkMax(FeederConstants.kFeederMotorCANId, MotorType.kBrushless);
        m_pivotMotor = new CANSparkMax(FeederConstants.kPivotMotorCANId, MotorType.kBrushless);

        m_beamBreakSensor = new DigitalInput(FeederConstants.kBeamBreakDIOId);

        m_feederMotor.restoreFactoryDefaults();
        m_feederMotor.setIdleMode(IdleMode.kBrake);

        m_pivotMotor.restoreFactoryDefaults();
        m_pivotMotor.setIdleMode(IdleMode.kBrake);

        m_pivotPID = m_pivotMotor.getPIDController();
        // set false to not pop off chain
        m_pivotPID.setPositionPIDWrappingEnabled(false);

        m_pivotEncoder = m_pivotMotor.getEncoder();

        // set pivot PID coefficients
        m_pivotPID.setP(FeederConstants.PivotPID.kP);
        m_pivotPID.setI(FeederConstants.PivotPID.kI);
        m_pivotPID.setD(FeederConstants.PivotPID.kD);
        m_pivotPID.setIZone(FeederConstants.PivotPID.kIz);
        m_pivotPID.setFF(FeederConstants.PivotPID.kFF);
        m_pivotPID.setOutputRange(FeederConstants.PivotPID.kMinOutput, FeederConstants.PivotPID.kMaxOutput);
    }

    @Override
    public void periodic() {
        m_pivotPID.setReference(m_pivotSetpoint, CANSparkMax.ControlType.kPosition);

        SmartDashboard.putNumber("pivot setpoint", m_pivotSetpoint);
        SmartDashboard.putNumber("pivot position", m_pivotEncoder.getPosition());
    }

    /**
     * Set the state of the feeder
     * 
     * @param state the new state for the feeder
     */
    public void setState(FeederState state) {
        m_state = state;

        double speed = switch (m_state) {
            case kReversed -> FeederConstants.kReverseSpeed;
            case kStopped -> 0;
            case kIntaking -> FeederConstants.kIntakeSpeed;
            case kShooting -> FeederConstants.kShootSpeed;
        };

        m_feederMotor.set(speed);
    }

    /**
     * Set the general mode of the shooter.
     * This specifies the available ranges for the pivot.
     * @param pos The new general position for the shooter
     */
    public void setPosition(FeederPivotPosition pos) {
        m_pivotPosition = pos;
        setPrecisePosition(m_pivotSetpoint);
    }

    /**
     * Set the specific setpoint for the feeder
     * 
     * @param setpoint the new setpoint for the shooter
     */
    public void setPrecisePosition(double setpoint) {
        if (Double.isNaN(setpoint)) {
            System.out.println("Got NAN pivot setpoint");
            return;
        }

        m_pivotSetpoint = Math.max(m_pivotPosition.lowLimit, Math.min(setpoint, m_pivotPosition.highLimit));
    }

    /**
     * Get the state of the beam break sensor
     * 
     * @return true if a note is detected, or fale otherwise
     */
    public boolean getBeamBreakState() {
        return !m_beamBreakSensor.get();
    }

    /**
     * set the state of the feeder
     * 
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
}
