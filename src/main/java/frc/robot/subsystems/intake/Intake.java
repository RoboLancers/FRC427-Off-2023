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
    CANSparkMax IntakeMotor = new CANSparkMax(Constants.IntakeConstants.IntakeMotorId, MotorType.kBrushless);
    RelativeEncoder IntakeEncoder = IntakeMotor.getEncoder();

    CANSparkMax IntakeMotor2 = new CANSparkMax(Constants.IntakeConstants.IntakeMotorId2, MotorType.kBrushless);
    RelativeEncoder IntakeEncoder2 = IntakeMotor.getEncoder();
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
        IntakeMotor.setInverted(Constants.IntakeConstants.IntakeInversionState1);
        IntakeMotor.setSmartCurrentLimit(Constants.IntakeConstants.IntakeMotorlimit);

        IntakeMotor2.setInverted(Constants.IntakeConstants.IntakeInversionState2);
        IntakeMotor2.setSmartCurrentLimit(Constants.IntakeConstants.IntakeMotorlimit);
        IntakeMotor2.follow(IntakeMotor);
    }

    public void periodic() {
        // code inside here will run repeatedly while the robot is on
        
    }
    public void intake(double speed){
        IntakeMotor.set(speed);
        IntakeMotor2.set(speed);
    }
    public void outtake(double speed){
        IntakeMotor.set(speed);
    }
    public void StopMotor(){
        IntakeMotor.set(0);
    }

    public void timer(Timer timer) {
    }
}
