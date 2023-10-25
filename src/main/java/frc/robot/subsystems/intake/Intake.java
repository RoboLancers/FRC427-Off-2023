package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

 /*
  * functionality/specs/notes: 
  - one motor for each side of the intake (two motors in total)
  - make one of the motors the main motor and have the other one follow it
  - be able to intake at a specific speed
  - be able to outtake at a specific speed
  - be able to stop the motor (stop intaking/outtaking)
  */

public class Intake extends SubsystemBase {
    // put motors & stuff here
    CANSparkMax m_intakeMotorRight = new CANSparkMax(Constants.IntakeConstants.kIntakeMotorRightId, MotorType.kBrushless);
    RelativeEncoder m_intakeEncoderRight = m_intakeMotorRight.getEncoder();

    CANSparkMax m_intakeMotorLeft = new CANSparkMax(Constants.IntakeConstants.kIntakeMotorLeftId, MotorType.kBrushless);
    RelativeEncoder m_intakeEncoderLeft = m_intakeMotorLeft.getEncoder();
    
    
    public Intake() {
    }

    public void setupMotors() {
        // set up motors
        /*
         * 
         (put all constants in Constants.java)
         - set motor inversion
         - set smart current limit (usually 40 is good)
         - set position & velocity conversion factor
         */
        m_intakeMotorRight.setInverted(Constants.IntakeConstants.kIntakeRightInversionState);
        m_intakeMotorRight.setSmartCurrentLimit(Constants.IntakeConstants.kIntakeMotorlimit);

        m_intakeMotorLeft.setInverted(Constants.IntakeConstants.kIntakeLeftInversionState);
        m_intakeMotorLeft.setSmartCurrentLimit(Constants.IntakeConstants.kIntakeMotorlimit);
        m_intakeMotorLeft.follow(m_intakeMotorRight);
    }

    public void periodic() {
        // code inside here will run repeatedly while the robot is on
        
    }
    public void intakeCube(double speed){
        m_intakeMotorRight.set(speed);
    }
    public void outtake(double speed){
        m_intakeMotorRight.set(-speed);
    }
    public void stopMotor(){
        m_intakeMotorRight.set(0);
    }
}
