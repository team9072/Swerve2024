package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.Constants.ArmConstants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ArmSubsystem extends SubsystemBase {
    private static final int kCanID = 13;
    private static final MotorType kMotorType = MotorType.kBrushless;

    private CANSparkMax m_motor;
    private SparkPIDController m_pidController;

    /**
     * An alternate encoder object is constructed using the GetAlternateEncoder()
     * method on an existing CANSparkMax object. If using a REV Through Bore
     * Encoder, the type should be set to quadrature and the counts per
     * revolution set to 8192
     */
    private RelativeEncoder m_encoder;

    public enum IntakerMode {
        INTAKE(ArmConstants.kIntakerSpeed),
        SHOOT(-ArmConstants.kShootSpeed),
        SHOOT_BOTTOM(-ArmConstants.kBottomShootSpeed),
        SHOOT_MIDDLE(-ArmConstants.kMiddleShootSpeed),
        SHOOT_TOP(-ArmConstants.kTopShootSpeed),
        IDLE(0);

        public double m_motorSpeed;

        IntakerMode(double motorSpeed) {
            m_motorSpeed = motorSpeed;
        }
    }

    public CANSparkMax m_intaker;
    public CANSparkMax m_followIntaker;

    // Public for testing encoders
    // public CANSparkMax m_rotation;

    public IntakerMode m_currentMode = IntakerMode.IDLE;

    public ArmSubsystem() {
        // initialize SPARK MAX with CAN ID
        m_motor = new CANSparkMax(kCanID, kMotorType);
        m_motor.restoreFactoryDefaults();
        m_motor.setSmartCurrentLimit(40); // new limit

        // set controller to use through-bore (absolute) encoder
        m_encoder = m_motor.getEncoder();
        m_pidController = m_motor.getPIDController();
        m_pidController.setFeedbackDevice(m_encoder);

        // set PID coefficients
        m_pidController.setP(1);
        m_pidController.setI(1e-4);
        m_pidController.setD(0);
        m_pidController.setIZone(0);
        m_pidController.setFF(0);
        m_pidController.setOutputRange(-ArmConstants.kRotationSpeed, ArmConstants.kRotationSpeed);

        m_intaker = new CANSparkMax(11, MotorType.kBrushless);
        m_followIntaker = new CANSparkMax(10, MotorType.kBrushless);
    }

    // Toggle the intaker
    // If it's going in, make it go out, and vise-versa
    public void toggleIntaker() {
        switch (m_currentMode) {
            case IDLE:
                setIntakerMode(IntakerMode.INTAKE);
                break;
            case INTAKE:
                setIntakerMode(IntakerMode.IDLE);
                break;
            default:
                break;
        }
    }

    public Command getToggleIntakerCommand() {
        return this.runOnce(() -> this.toggleIntaker());
    }

    public Command getShootCommand(IntakerMode mode) {
        return this.runOnce(() -> this.setIntakerMode(mode))
                .andThen(
                        new WaitCommand(0.25),
                        this.runOnce(() -> this.setIntakerMode(IntakerMode.IDLE)))
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    private void setIntakerMode(IntakerMode mode) {
        this.m_currentMode = mode;
        m_intaker.set(m_currentMode.m_motorSpeed);
    }

    // // Set the rotation of the arm using the joystick value
    // public void setRotationSpeed(double speed) {
    // m_rotation.set(-Math.max(Math.min(speed, ArmConstants.kRotationSpeed),
    // -ArmConstants.kRotationSpeed));
    // }

    // Toggle the intaker
    // If it's going in, make it go out, and vise-versa
    public void ArmShootPosition() {
        switch (m_currentMode) {
            case IDLE:
                setIntakerMode(IntakerMode.INTAKE);
                break;
            case INTAKE:
                setIntakerMode(IntakerMode.IDLE);
                break;
            default:
                break;
        }
    }

    public Command setArmPosition(double rotations) {
        return this.runOnce(() -> m_pidController.setReference(rotations, CANSparkMax.ControlType.kPosition));
    }
}