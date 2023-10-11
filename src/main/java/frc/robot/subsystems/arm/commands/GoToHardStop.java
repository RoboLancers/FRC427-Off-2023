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
        // arm goes to initial angle
        m_arm.goToAngle(Constants.ArmConstants.kInitialAngle);


    }

    public void execute() {
    }

    public boolean isFinished() {
        // confirms arm is at initial angle
        return m_arm.isAtAngle(); 
    }

    public void end(boolean interrupted) {
        // runs when the command is ended
    }
}
