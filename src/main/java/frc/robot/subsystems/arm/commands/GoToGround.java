package frc.robot.subsystems.arm.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm;

public class GoToGround extends CommandBase{
    Arm m_arm;
  

    public GoToGround(Arm arm) {
        m_arm = arm;
    

        addRequirements(arm);
    }

    public void initialize() {
        // makes arm go to ground angle
        m_arm.goToAngle(Constants.ArmConstants.kGroundAngle);


    }

    public void execute() {
    }

    public boolean isFinished() {
        // makes sure arm is at ground angle
        return m_arm.isAtAngle(); 
    }

    public void end(boolean interrupted) {
        // runs when the command is ended
    }

    
}
