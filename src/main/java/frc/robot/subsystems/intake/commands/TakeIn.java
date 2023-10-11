package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.intake.Intake;

public class TakeIn extends CommandBase {
     // declare how long to intake for and speed
    Intake m_intake;
    double m_speed;

     // establishes intake, speed,
    public TakeIn(Intake intake, double speed) {
    this.m_intake = intake;
    this.m_speed = speed;

        addRequirements(intake);
    }
    // starts intaking 
    public void initialize() {
        // runs when the command is FIRST STARTED
        this.m_intake.intakeCube(m_speed);
    }
    // keeps intaking
    public void execute() {
        // runs repeatedly until the command is finished
    }
    // checks to stops intaking
    public boolean isFinished() {
        // runs and tells whether or not the command should finish
        return true;
    }
    // stops intaking
    public void end(boolean interrupted) {
        // runs when the command is ended
    }
}
