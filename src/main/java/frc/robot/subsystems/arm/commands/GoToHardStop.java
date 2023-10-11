package frc.robot.subsystems.arm.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm;

public class GoToHardStop extends CommandBase {
    Arm m_arm;
  

    public GoToHardStop(Arm arm) {
        m_arm = arm;
    

        addRequirements(arm);
    }

    public void initialize() {
        // runs when the command is FIRST STARTED
        m_arm.goToAngle(Constants.ArmConstants.kInitialAngle);


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
