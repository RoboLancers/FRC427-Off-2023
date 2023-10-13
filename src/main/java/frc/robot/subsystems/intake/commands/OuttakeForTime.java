package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.Intake;

public class OuttakeForTime extends CommandBase {
        // declare how long to outtake for and how fast
    Intake m_intake;
    double m_speed;

    Timer m_timer = new Timer();
    private double m_time;
        // establishes outtake, speed, and time
    public OuttakeForTime(Intake outtake, double speed, double time) {
    this.m_intake = outtake;
    this.m_speed = speed;
    this.m_time = time;

        addRequirements(outtake);
    }

    public void initialize() {
        // runs when the command is FIRST STARTED
        //outtakes cube for at a certain speed and declares timer starting
        this.m_intake.outtake(m_speed);     
        m_timer.start();
    }   

    public void execute() {
         // runs repeatedly until the command is finished
    
    
    }

    public boolean isFinished() {
        // runs and tells whether or not the command should finish
        return this.m_timer.hasElapsed(m_time);
    }

    public void end(boolean interrupted) {
        // runs when the command is ended
        // stops motor
        this.m_intake.stopMotor();
    }
}
