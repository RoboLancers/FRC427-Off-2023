package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

// represents a single swerve pod on the robot
public class SwerveModule {

    // current target state (speed, angle) of the swerve pod
    private SwerveModuleState targetState;
    
    // the turn and drive motors of the swerve pod
    private CANSparkMax turnMotor; 
    private CANSparkMax driveMotor;
    
    // PID controllers for each respective motor
    public SwerveTurnPIDController turnPIDController; 
    private SparkMaxPIDController drivePIDController; 

    // encoder for the angle of the wheel relative to 0 degrees (forward)
    private CANCoder absoluteTurnEncoder;

    // encoder for the drive wheel
    private RelativeEncoder driveEncoder; 

    // feedforward values of the drive, not necessarily needed
    private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(
        Constants.DrivetrainConstants.ksVolts, 
        Constants.DrivetrainConstants.kvVoltSecondsPerMeter, 
        Constants.DrivetrainConstants.kaVoltSecondsSquaredPerMeter
    ); 

    /**
     * 
     * @param kTurn CAN ID of the turn motor
     * @param kDrive CAN ID of the drive motor
     * @param kTurnEncoder CAN ID of the turning CANEncoder
     */
    public SwerveModule(int kTurn, int kDrive, int kTurnEncoder) {
        this.turnMotor = new CANSparkMax(kTurn, MotorType.kBrushless); 
        this.driveMotor = new CANSparkMax(kDrive, MotorType.kBrushless); 

        this.absoluteTurnEncoder = new CANCoder(kTurnEncoder); 
        this.driveEncoder = this.driveMotor.getEncoder(); 

        this.turnPIDController = new SwerveTurnPIDController(absoluteTurnEncoder, 0, 0, 0); 
        this.drivePIDController = this.driveMotor.getPIDController(); 

        configureMotors();
        configureEncoders();
        configurePIDControllers();

        // in case of brownout
        this.turnMotor.burnFlash(); 
        this.driveMotor.burnFlash(); 
    }

    // Sets current limits, idle modes, etc. for each motor for maximum performance
    private void configureMotors() {
        this.driveMotor.setSmartCurrentLimit(Constants.DrivetrainConstants.kDriveCurrentLimit); 
        this.driveMotor.setIdleMode(IdleMode.kBrake); 
        this.driveMotor.enableVoltageCompensation(12); 

        this.turnMotor.setSmartCurrentLimit(Constants.DrivetrainConstants.kTurnCurrentLimit); 
        this.turnMotor.setIdleMode(IdleMode.kBrake); 
        this.turnMotor.enableVoltageCompensation(12); 
    }

    // sets the conversion factors for the drive encoder based on gear ratios
    private void configureEncoders() {
        this.driveEncoder.setPositionConversionFactor(Constants.DrivetrainConstants.kMetersPerRot);
        this.driveEncoder.setVelocityConversionFactor(Constants.DrivetrainConstants.kMetersPerSecondPerRPM); 
    }

    // sets the PID constants for turn & drive
    private void configurePIDControllers() {
        setTurnPID(
            Constants.DrivetrainConstants.kModuleTurn_P, 
            Constants.DrivetrainConstants.kModuleTurn_I, 
            Constants.DrivetrainConstants.kModuleTurn_D
            );
        
        setDrivePID(Constants.DrivetrainConstants.kModuleDrive_P, Constants.DrivetrainConstants.kModuleDrive_I, Constants.DrivetrainConstants.kModuleDrive_D, Constants.DrivetrainConstants.kModuleDrive_FF);
    }

    // sets the PID constants for the turn motor
    public void setTurnPID(double p, double i, double d) {
        this.turnPIDController.setPID(p, i, d);
    }

    // sets the PID constants for the drive motor
    public void setDrivePID(double p, double i, double d, double ff) {
        this.drivePIDController.setP(p); 
        this.drivePIDController.setI(i); 
        this.drivePIDController.setD(d); 
        this.drivePIDController.setFF(ff); 
    }

    /**
     * 
     * @param state the target speed & angle for the module to go to, in the form of a {@link SwerveModuleState}
     * @param driveType the type of drive to operate with
     */
    public void updateState(SwerveModuleState state, DriveState driveType) {
        SmartDashboard.putNumber("module " + absoluteTurnEncoder.getDeviceID() + " desired speed", state.speedMetersPerSecond); 
        SmartDashboard.putNumber("module " + absoluteTurnEncoder.getDeviceID() + " actual speed", this.driveEncoder.getVelocity()); 
        SmartDashboard.putNumber("module " + absoluteTurnEncoder.getDeviceID() + " diff speed", state.speedMetersPerSecond - this.driveEncoder.getVelocity()); 

        this.targetState = state; 

        // optimize angles so the wheels only have to turn 90 degrees to reach their setpoint at any given time
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state, getAngle()); 

        if (driveType == DriveState.OPEN_LOOP) updateOpenLoopDriveState(optimizedState.speedMetersPerSecond); 
         else updateClosedLoopDriveState(optimizedState.speedMetersPerSecond);

         updateTurnState(optimizedState.angle);
    }

    // handle open loop drive; calculate # output based on theoretical motor maximums
    private void updateOpenLoopDriveState(double speed) {
        double percent = MathUtil.clamp(speed / Constants.DrivetrainConstants.kMaxAttainableSpeedMetersPerSecond, -1, 1); 
        driveMotor.set(percent);
    }
    
    // handle closed loop drive; use SparkMAX's PID controller to go to the specified speed
    private void updateClosedLoopDriveState(double speed) {
        drivePIDController.setReference(speed, ControlType.kVelocity, 0, driveFeedforward.calculate(speed)); 
    }

    /**
     * Turns the pod to the desired angle 
     * @param turn the desired angle to turn to
     */
    private void updateTurnState(Rotation2d turn) {
        turnPIDController.setSetpoint(turn.getDegrees()); 
        turnMotor.set(turnPIDController.calculate());
    }

    // current angle of the swerve pod
    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(this.absoluteTurnEncoder.getAbsolutePosition()); 
    }

    // current state (velocity & angle) of the swerve pod
    public SwerveModuleState getCurrentState() {
        return new SwerveModuleState(this.driveEncoder.getVelocity(), getAngle()); 
    }

    // current position (position & angle) of the swerve pod
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(this.driveEncoder.getPosition(), getAngle()); 
    }

    // target state (velocity & angle) of the swerve pod
    public SwerveModuleState getReferenceState() {
        return this.targetState; 
    }

    public static enum DriveState {
        OPEN_LOOP, // drive the motor purely with calculations of how fast it should do; does not take into account resistance
        CLOSED_LOOP // discretely controls the motor's speed based on past motor speed data using PID
    }
}
