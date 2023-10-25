package frc.robot;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.Balance;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.commands.GoToGround;
import frc.robot.subsystems.arm.commands.GoToHardStop;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.IntakeForTime;
import frc.robot.subsystems.intake.commands.OuttakeForTime;

// class to store, set up, and choose autos
public class AutoPicker extends CommandBase  {
    private SendableChooser<Command> chooser = new SendableChooser<>();

    private SwerveAutoBuilder autoBuilder;

    Arm m_ArmSubsystem;
    Intake m_intakeSubsystem;
    Drivetrain m_driveSubsystem;
    
    public AutoPicker(Drivetrain driveSubsystem, Intake intakeSubsystem, Arm ArmSubsystem) {
        m_ArmSubsystem = ArmSubsystem;
        m_intakeSubsystem = intakeSubsystem;
        m_driveSubsystem = driveSubsystem;

        addRequirements(driveSubsystem);
        addRequirements(intakeSubsystem);
        addRequirements(driveSubsystem);

        // see PathPlanner
        
        autoBuilder = new SwerveAutoBuilder(
            driveSubsystem::getPose, // Pose2d supplier
            driveSubsystem::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
            Constants.DrivetrainConstants.kDriveKinematics, // SwerveDriveKinematics
            new PIDConstants(Constants.Trajectory.kDrive_P, Constants.Trajectory.kDrive_I, Constants.Trajectory.kDrive_D), // PID constants to correct for translation error (used to create the X and Y PID controllers)
            new PIDConstants(Constants.Trajectory.kOmega_P, Constants.Trajectory.kOmega_I, Constants.Trajectory.kOmega_D), // PID constants to correct for rotation error (used to create the rotation controller)
            driveSubsystem::swerveDrive, // Module states consumer used to output to the drive subsystem
            Constants.Trajectory.eventMap,
            true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            driveSubsystem // The drive subsystem. Used to properly set the requirements of path following commands
        );


    }

    public void addEvents() {
        // eg. addEvent("intake_cube", new IntakeForTime(intake, 1, 2)); 
        addEvent("score_cube", new OuttakeForTime(m_intakeSubsystem, Constants.IntakeConstants.kShootSpeedHigh, 2));
        addEvent("put_arm_down", new GoToGround(m_ArmSubsystem));
        addEvent("put_arm up", new GoToHardStop(m_ArmSubsystem));
        addEvent("intake_cube", new IntakeForTime(m_intakeSubsystem, Constants.IntakeConstants.kIntakeSpeed, 2));
        addEvent("balance_auto", new Balance(m_driveSubsystem));

    }

    public void addAutos() {
        // eg. addPPSwerveAuto("BalanceAuto", "Mid Lane Auto"); 
        addPPSwerveAuto("FirstAuto", "TwoCubesAuto");
        addPPSwerveAuto("SecondAuto", "BalanceAuto");
        addPPSwerveAuto("ThirdAuto", "TaxiAuto");
        addPPSwerveAuto("FourthAuto", "TestAuto");
    }

    public void addEvent(String key, Command command) {
        Constants.Trajectory.eventMap.put(key, command); 
    }

    // add an arbitrary auto as a command
    public void addAuto(String name, Command auto) {
        this.chooser.addOption(name, auto);
    }

    // add a path planner auto based on file name
    public void addPPSwerveAuto(String name, String fileName, double maxVel, double maxAccel) {
        List<PathPlannerTrajectory> group = PathPlanner.loadPathGroup(fileName, new PathConstraints(maxVel, maxAccel)); 

        addAuto(name, autoBuilder.fullAuto(group)); 
    }

    // add a pathplanner auto based on file name with default acceleration and velocity constraints
    public void addPPSwerveAuto(String name, String fileName) {
        addPPSwerveAuto(name, fileName, Constants.Trajectory.kMaxVelocityMetersPerSecond, Constants.Trajectory.kMaxAccelerationMetersPerSecondSquared); 
    }

    // gets the currently selected auto
    public Command getAuto() {
        return chooser.getSelected(); 
    }
}
