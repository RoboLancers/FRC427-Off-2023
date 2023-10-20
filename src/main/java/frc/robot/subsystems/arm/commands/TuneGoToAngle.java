package frc.robot.subsystems.arm.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.util.IOUtils;

public class TuneGoToAngle extends CommandBase {
    Arm m_arm;
    double m_angle;

    public TuneGoToAngle(Arm arm) {
        m_arm = arm;
        m_angle = IOUtils.get("ArmAngle");

        addRequirements(arm);

    }

    public void initialize() {
        // makes arm go to angle
        m_arm.goToAngle(m_angle);
        
        

    }

    public void execute() {

    }

    public boolean isFinished() {
        // confirmation that arm is at angle
        return m_arm.isAtAngle(); 
    }

    public void end(boolean interrupted) {
        // runs when the command is ended
    }
}



//Make arm go to angle
//arm to hard stop, arm to ground 