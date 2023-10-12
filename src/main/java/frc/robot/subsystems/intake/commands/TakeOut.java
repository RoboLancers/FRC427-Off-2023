package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.intake.Intake;

public class TakeOut extends CommandBase {
    Intake intake;
    double speed;

    public TakeOut(Intake intake, double speed) {
    this.intake = intake;
    this.speed = speed;

        addRequirements(intake);
        
    }

    public void initialize() {
        // runs when the command is FIRST STARTED
        this.intake.outtake(speed);
        
    }

    public void execute() {
        // runs repeatedly until the command is finished
    }

    public boolean isFinished() {
        // runs and tells whether or not the command should finish
        return true;
    }

    public void end(boolean interrupted) {
        // runs when the command is ended
    }
}
