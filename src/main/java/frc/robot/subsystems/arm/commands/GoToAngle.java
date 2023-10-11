package frc.robot.subsystems.arm.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;

public class GoToAngle extends CommandBase {
    Arm m_arm;
    double m_angle;

    public GoToAngle(Arm arm, double angle) {
        m_arm = arm;
        m_angle = angle;

        addRequirements(arm);

    }

    public void initialize() {
        // runs when the command is FIRST STARTED
        m_arm.goToAngle(m_angle);
        

    }

    public void execute() {
        // runs repeatedly until the command is finished
    }

    public boolean isFinished() {
        // runs and tells whether or not the command should finish
        return m_arm.isAtAngle(); 
    }

    public void end(boolean interrupted) {
        // runs when the command is ended
    }
}



//Make arm go to angle
//arm to hard stop, arm to ground 