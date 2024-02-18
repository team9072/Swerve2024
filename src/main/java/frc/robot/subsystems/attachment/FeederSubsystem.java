package frc.robot.subsystems.attachment;

import com.revrobotics.CANSparkLowLevel.MotorType;

import javax.print.attribute.standard.MediaSize.NA;

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
        kIntakePosition,
        kAmpPosition,
        kSpeakerPosition;
    }

    private final CANSparkMax m_feederMotor;
    private final CANSparkMax m_pivotMotor;
    private SparkPIDController m_pivotPID;
    private RelativeEncoder m_pivotEncoder;

    private final DigitalInput m_beamBreakSensor;

    private FeederState m_state = FeederState.kStopped;
    private FeederPivotPosition m_position = FeederPivotPosition.kIntakePosition;
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

        // display PID coefficients on SmartDashboard
        SmartDashboard.putNumber("P Gain", FeederConstants.PivotPID.kP);
        SmartDashboard.putNumber("I Gain", FeederConstants.PivotPID.kI);
        SmartDashboard.putNumber("D Gain", FeederConstants.PivotPID.kD);
        SmartDashboard.putNumber("I Zone", FeederConstants.PivotPID.kIz);
        SmartDashboard.putNumber("Feed Forward", FeederConstants.PivotPID.kFF);
        SmartDashboard.putNumber("Max Output", FeederConstants.PivotPID.kMaxOutput);
        SmartDashboard.putNumber("Min Output", FeederConstants.PivotPID.kMinOutput);
        SmartDashboard.putNumber("Set Rotations", 0);
    }

    @Override
    public void periodic() {
        double kP = 0, kI = 0, kD = 0, kIz = 0, kFF = 0, kMinOutput = 0, kMaxOutput = 0;

        // read PID coefficients from SmartDashboard
        double p = SmartDashboard.getNumber("P Gain", 0);
        double i = SmartDashboard.getNumber("I Gain", 0);
        double d = SmartDashboard.getNumber("D Gain", 0);
        double iz = SmartDashboard.getNumber("I Zone", 0);
        double ff = SmartDashboard.getNumber("Feed Forward", 0);
        double max = SmartDashboard.getNumber("Max Output", 0);
        double min = SmartDashboard.getNumber("Min Output", 0);

        if ((p != kP)) {
            m_pivotPID.setP(p);
            kP = p;
        }
        if ((i != kI)) {
            m_pivotPID.setI(i);
            kI = i;
        }
        if ((d != kD)) {
            m_pivotPID.setD(d);
            kD = d;
        }
        if ((iz != kIz)) {
            m_pivotPID.setIZone(iz);
            kIz = iz;
        }
        if ((ff != kFF)) {
            m_pivotPID.setFF(ff);
            kFF = ff;
        }
        if ((max != kMaxOutput) || (min != kMinOutput)) {
            m_pivotPID.setOutputRange(min, max);
            kMinOutput = min;
            kMaxOutput = max;
        }

        m_pivotPID.setReference(m_pivotSetpoint, CANSparkMax.ControlType.kPosition);

        SmartDashboard.putNumber("SetPoint", m_pivotSetpoint);
        SmartDashboard.putNumber("ProcessVariable", m_pivotEncoder.getPosition());
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

    public void setPosition() {

    }

    /**
     * Set the specific setpoint for the feeder
     * @param setpoint the new setpoint for the shooter
     */
    public void setPrecisePosition(double setpoint) {
        if (Double.isNaN(setpoint)) { System.out.println("Got NAN pivot setpoint"); return; }

        m_pivotSetpoint = Math.max(0,  Math.min(setpoint, 60));
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
