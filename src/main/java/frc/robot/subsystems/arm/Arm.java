package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    double m_targetPosition = Constants.ArmConstants.kInitialAngle;
    double m_velocity = 0;
    

    //Initializing motors; defining encoders; defining PID controllers; defining feedforward
    CANSparkMax m_armMotor = new CANSparkMax(Constants.ArmConstants.kArmMotorId, MotorType.kBrushless);
    
    RelativeEncoder m_armEncoder = m_armMotor.getEncoder();
    public SparkMaxPIDController m_ArmPIDController = m_armMotor.getPIDController();
    public ArmFeedforward m_feedForward = new ArmFeedforward(Constants.ArmConstants.kS, Constants.ArmConstants.kV, Constants.ArmConstants.kG, Constants.ArmConstants.kA);
    

    public Arm() {
        // calculations for feedforward
        setupMotors();
    }

    public void setupMotors() {
        m_armMotor.setInverted(true);
        // note motor counts rotations
        m_armMotor.setSmartCurrentLimit(Constants.ArmConstants.kCurrentLimit);
        m_armEncoder.setPositionConversionFactor(Constants.ArmConstants.kConversionFactor);
        // to get the value below, divide this value (0.5) by 60
        m_armEncoder.setVelocityConversionFactor(Constants.ArmConstants.kVelocityConversionFactor);
        m_armEncoder.setPosition(Constants.ArmConstants.kInitialAngle);
        //setup PID :)
        m_ArmPIDController.setP(Constants.ArmConstants.kP);
        m_ArmPIDController.setD(Constants.ArmConstants.kD);
        m_ArmPIDController.setI(0);

    }
    
    public void goToAngle(double position){
        // sets position to target position; feedforward calculations for position and velocity
        this.m_targetPosition = position;
    }

    public void periodic() {
        // gets velocity + more calculations for PID
        m_ArmPIDController.setReference(this.m_targetPosition, ControlType.kPosition, 0, m_feedForward.calculate(Math.toRadians(m_armEncoder.getPosition()), m_velocity));
        this.m_velocity = m_armEncoder.getVelocity();
    }
    
   public boolean isAtAngle() {
    // checks if position is good or not
    return (Math.abs(m_armEncoder.getPosition()) < Constants.ArmConstants.kAngleError);
   }
   public double getAngle() {
    return m_armEncoder.getPosition();
   }
}
