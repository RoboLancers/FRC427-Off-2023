package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

 /*
  * functionality/specs/notes: 
  - one motor for each side of the intake (two motors in total)+
  - make one of the motors the main motor and have the other one follow it+
  - be able to intake at a specific speed+
  - be able to outtake at a specific speed+
  - be able to stop the motor (stop intaking/outtaking)+
  */

public class Intake extends SubsystemBase {
    // put motors & stuff here
    CANSparkMax m_intakeMotor = new CANSparkMax(Constants.IntakeConstants.IntakeMotorId, MotorType.kBrushless);
    RelativeEncoder m_intakeEncoder = m_intakeMotor.getEncoder();

    CANSparkMax m_intakeMotor2 = new CANSparkMax(Constants.IntakeConstants.IntakeMotorId2, MotorType.kBrushless);
    RelativeEncoder m_intakeEncoder2 = m_intakeMotor.getEncoder();
    public Intake() {

    }

    public void setupMotors() {
        // set up motors    
        /*
         * 
         (put all constants in Constants.java)
         - set motor inversion
         - set smart current limit (usually 40 is good)
         - set position & velocity conversion factor (later)
         */
        m_intakeMotor.setInverted(Constants.IntakeConstants.IntakeInversionState1);
        m_intakeMotor.setSmartCurrentLimit(Constants.IntakeConstants.IntakeMotorlimit);

        m_intakeMotor2.setInverted(Constants.IntakeConstants.IntakeInversionState2);
        m_intakeMotor2.setSmartCurrentLimit(Constants.IntakeConstants.IntakeMotorlimit);
        m_intakeMotor2.follow(m_intakeMotor);
    }

    public void periodic() {
        // code inside here will run repeatedly while the robot is on
        
    }
    public void intakeCube(double speed){
        m_intakeMotor.set(speed);
    }
    public void outtake(double speed){
        m_intakeMotor.set(-speed);
    }
    public void stopMotor(){
        m_intakeMotor.set(0);
    }
}
