package frc.robot.subsystems.arm;

import javax.swing.text.Position;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

 /*
  * functionality/specs/notes: 
  - one single motor turns the arm
  - when robot is first turned on, set the arm encoder to a specific angle

  - make arm go to a specific target angle
  double motorTargetAngle = 90.0;
  

  - be able to get the current angle of the motor

  - be able to check whether or not motor is at target angle // note put proportion in constants; power = error(distance) x P (constant)
boolean motorReachesTargetAngle;
if (motorReachesTargetAngle) {
    System.out.println(motorReachesTargetAngle)
}

  - convention: arm flat - 0 degrees, arm bent backwards - ~150 degrees
  double armFlat = 0.0;
  double armBentBackwards = 150.0;
  */

public class Arm extends SubsystemBase {
    double targetPosition = 0;

    // put motors & stuff here
    CANSparkMax armMotor = new CANSparkMax(0, MotorType.kBrushless);
    RelativeEncoder armEncoder = armMotor.getEncoder();
    SparkMaxPIDController armPIDController = armMotor.getPIDController();
    

    public Arm() {
        setupMotors();
    }

    public void setupMotors() {
        // set up motors / runs when motor first turns on
        /*
         * 
         (put all constants in Constants.java)
         - set motor inversion
         - set smart current limit (usually 40 is good)
         - set position & velocity conversion factor
         */

         armMotor.setInverted(true);
         // note motor counts rotations
         armMotor.setSmartCurrentLimit(Constants.ArmConstants.limit);
         armEncoder.setPositionConversionFactor(Constants.ArmConstants.conversionFactor);
         // to get the value below, divide this value (0.5) by 60
         armEncoder.setVelocityConversionFactor(Constants.ArmConstants.velocityconversionFactor);
// thingy for encoder angle
         armEncoder.setPosition(Constants.ArmConstants.initialAngle);
         armPIDController.setP(Constants.ArmConstants.p);
         armPIDController.setI(0);
         armPIDController.setD(0);
         



            // thingy for motor angle
           

// thingy for convention
double armFlat = 0.0;
double armBentBackwards = 150.0;
            
            
        } 

        
    

    

    public void setupMotorPID() {
        // setup motors pid
        /*
         * (put all PID constants in Constants.java)
         
         (ask jason once you get here if you don't understand)
         
         - set P, I, D values of the motor
         - set ArmFeedforward values
         */
    }

    public void periodic() {
        // code inside here will run repeatedly while the robot is on
        // note speed is between -1.0 to 1.0

    }
    public void goToAngle(double position){
        this.targetPosition = position;
        armPIDController.setReference(position, ControlType.kPosition);
        

    }
   public boolean isAtAngle(){
    if (armEncoder.getPosition()<targetPosition+10 && armEncoder.getPosition()>targetPosition-10) {
        return true;
    }
    else {
        return false;
    }
   }
   
}
