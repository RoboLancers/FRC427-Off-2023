package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
    }

    public void periodic() {
        // code inside here will run repeatedly while the robot is on

    }
}
