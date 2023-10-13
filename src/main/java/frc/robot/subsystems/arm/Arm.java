package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

 /*
  * functionality/specs/notes: 
  - one single motor turns the arm
  - when robot is first turned on, set the arm encoder to a specific angle
  - make arm go to a specific target angle
  - be able to get the current angle of the motor
  - be able to check whether or not motor is at target angle

  - convention: arm flat - 0 degrees, arm bent backwards - ~150 degrees
  */

public class Arm extends SubsystemBase {
    // put motors & stuff here

    public Arm() {
        setupMotors();
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

    }
}
