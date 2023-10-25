package frc.robot.subsystems.intake.commands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.IOUtils;

// declare how long to intake for and how fast
public class TuneIntakeForTime extends CommandBase {

    Intake m_intake;
    double m_speed;

    Timer m_timer = new Timer();
    private double m_time;

    // establishes intake, speed, and time
    public TuneIntakeForTime(Intake intake) {
    this.m_intake = intake;
    this.m_speed = IOUtils.get("Tune_Intake_Speed");
    this.m_time = IOUtils.get("Tune_Intake_Time");

    addRequirements(intake);

    }

    public void initialize() {
        // runs when the command is FIRST STARTED
        //intakes cube for at a certain speed and declares timer starting
        this.m_intake.intakeCube(m_speed);     
        m_timer.start();
    }   

    public void execute() {
         // runs repeatedly until the command is finished
    
    }

    public boolean isFinished() {
        // runs and tells whether or not the command should finish
      return this.m_timer.hasElapsed(m_time);

        // return this.m_timer.hasElapsed(m_time);
       
    }

    public void end(boolean interrupted) {
        // runs when the command is ended
        // stops motor
        this.m_intake.stopMotor();
    }
}