package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.Intake;

public class TakeOut extends CommandBase {
       // declare to outtake and speed
    Intake m_intake;
    double m_speed;

    // establishes outtake, speed,
    public TakeOut(Intake intake, double speed) {
    this.m_intake = intake;
    this.m_speed = speed;

        addRequirements(intake);
        
    }
    // starts outtaking
    public void initialize() {
        // runs when the command is FIRST STARTED
        this.m_intake.outtake(m_speed);
        
    }
    // keeps outtaking
    public void execute() {
        // runs repeatedly until the command is finished
    }
    //checks to stop outtaking
    public boolean isFinished() {
        // runs and tells whether or not the command should finish
        return true;
    }
    // stops outtaking
    public void end(boolean interrupted) {
        // runs when the command is ended
    }
}
