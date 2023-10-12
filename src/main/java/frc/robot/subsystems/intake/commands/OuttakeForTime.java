package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.intake.Intake;

public class OuttakeForTime extends CommandBase {
  
    Outtake outtake;
    double speed;

    Timer timer = new Timer();
    private double time;
    public OuttakeForTime(Outtake outtake, double speed, double time) {
    this.outtake = outtake;
    this.speed = speed;
    this.time = time;

        addRequirements(outtake);
    }

    public void initialize() {
        // runs when the command is FIRST STARTED
        this.outtake.intake(speed);     
        timer.start();
    }   

    public void execute() {
         // runs repeatedly until the command is finished
    
    
    }

    public boolean isFinished() {
        // runs and tells whether or not the command should finish
       if (this.timer.hasElapsed(time)) {
        return true;
       } else {
        return false;
       }
    }

    public void end(boolean interrupted) {
        // runs when the command is ended
        this.outtake.StopMotor();
    }
}
