package frc.robot.subsystems.drivetrain;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.SwerveModule.DriveState;
import frc.robot.util.ChassisState;
import frc.robot.util.SwerveUtils;

public class Drivetrain extends SubsystemBase {

  // set up the four swerve modules  
  private SwerveModule frontLeft = new SwerveModule(Constants.DrivetrainConstants.FrontLeft.kRotate, Constants.DrivetrainConstants.FrontLeft.kDrive, Constants.DrivetrainConstants.FrontLeft.kRotEncoder); 
  private SwerveModule frontRight = new SwerveModule(Constants.DrivetrainConstants.FrontRight.kRotate, Constants.DrivetrainConstants.FrontRight.kDrive, Constants.DrivetrainConstants.FrontRight.kRotEncoder); 
  private SwerveModule backLeft = new SwerveModule(Constants.DrivetrainConstants.BackLeft.kRotate, Constants.DrivetrainConstants.BackLeft.kDrive, Constants.DrivetrainConstants.BackLeft.kRotEncoder); 
  private SwerveModule backRight = new SwerveModule(Constants.DrivetrainConstants.BackRight.kRotate, Constants.DrivetrainConstants.BackRight.kDrive, Constants.DrivetrainConstants.BackRight.kRotEncoder); 

  // initialize swerve position estimator
  private SwerveDrivePoseEstimator odometry; 

  // initialize the gyro on the robot
  public AHRS gyro = new AHRS(SPI.Port.kMXP);

  // represents the current drive state of the robot
  private DriveState driveState = DriveState.CLOSED_LOOP; 

  private PIDController rotationController = new PIDController(
      Constants.DrivetrainConstants.kTurn_P,
      Constants.DrivetrainConstants.kTurn_I,
      Constants.DrivetrainConstants.kTurn_D
  ); 

  public Drivetrain() {

    this.rotationController.enableContinuousInput(-180, 180); 

    // zero yaw when drivetrain first starts up
    this.gyro.zeroYaw();

    // create the pose estimator
    this.odometry = new SwerveDrivePoseEstimator(Constants.DrivetrainConstants.kDriveKinematics, gyro.getRotation2d(), getPositions(), new Pose2d()); 
    
  }

  @Override
  public void periodic() {
    // update the odometry with the newest rotations, positions of the swerve modules
    SmartDashboard.putNumber("drive yaw", gyro.getYaw());
    SmartDashboard.putNumber("drive x", odometry.getEstimatedPosition().getX());
    SmartDashboard.putNumber("drive y", odometry.getEstimatedPosition().getY());
    SmartDashboard.putNumber("drive omega", odometry.getEstimatedPosition().getRotation().getDegrees());
    
    this.odometry.update(gyro.getRotation2d(), getPositions());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  
  /**
   * 
   * Drive the robot with a forward, sidewards, rotation speed
   * 
   * @param xMetersPerSecond the speed to drive forward with in meters per second
   * @param yMetersPerSecond the speed to drive sidewards with in meters per second
   * @param rotationRadPerSecond the speed to rotate at in radians per second
   * 
   */
  public void swerveDrive(double xMetersPerSecond, double yMetersPerSecond, double rotationRadPerSecond) {
    swerveDrive(new ChassisSpeeds(
      xMetersPerSecond, 
      yMetersPerSecond, 
      rotationRadPerSecond
    ));
  }

  public void swerveDrive(ChassisSpeeds speeds) {
    ChassisSpeeds robotRelative = ChassisSpeeds.fromFieldRelativeSpeeds(
      speeds, 
      gyro.getRotation2d()
    ); 

    // correct for drift in the chassis
    ChassisSpeeds correctedSpeeds = SwerveUtils.correctInputWithRotation(robotRelative); 

    // calculate module states from the target speeds
    SwerveModuleState[] states = Constants.DrivetrainConstants.kDriveKinematics.toSwerveModuleStates(correctedSpeeds); 

    // ensure all speeds are reachable by the wheel
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.DrivetrainConstants.kMaxAttainableSpeedMetersPerSecond);

    swerveDrive(states);
  }
  public void swerveDriveRobotCentric(ChassisSpeeds speeds) {
    // correct for drift in the chassis
    ChassisSpeeds correctedSpeeds = SwerveUtils.correctInputWithRotation(speeds); 

    // calculate module states from the target speeds
    SwerveModuleState[] states = Constants.DrivetrainConstants.kDriveKinematics.toSwerveModuleStates(correctedSpeeds); 

    // ensure all speeds are reachable by the wheel
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.DrivetrainConstants.kMaxAttainableSpeedMetersPerSecond);

    swerveDrive(states);
  }
  private double lastTurnedTheta = 0; 

  public void resetLastTurnedTheta() {
    lastTurnedTheta = this.getYaw(); 
  }

  public void swerveDriveFieldRel(double xMetersPerSecond, double yMetersPerSecond, double thetaDegrees, boolean turn) {
    
   // if (turn) lastTurnedTheta = thetaDegrees; 
    
    // always make an effort to rotate to the last angle we commanded it to

    // to commit to going to our angle even after stopping pressing
   // double rotSpeed = rotationController.calculate(this.getYaw(), lastTurnedTheta); 

    // or to not commit to the angle
     if (turn) lastTurnedTheta = this.getYaw(); 
     double rotSpeed = rotationController.calculate(this.getYaw(), turn ? thetaDegrees : lastTurnedTheta); 
     

    rotSpeed = MathUtil.clamp(rotSpeed, -Constants.DrivetrainConstants.kMaxRotationRadPerSecond, Constants.DrivetrainConstants.kMaxRotationRadPerSecond); 

    swerveDrive(xMetersPerSecond, yMetersPerSecond, rotSpeed);
  }

  public void swerveDriveFieldRel(ChassisState state) {
    swerveDriveFieldRel(state.vxMetersPerSecond, state.vyMetersPerSecond, Math.toDegrees(state.omegaRadians), state.turn);
  }

  // command the swerve modules to the intended states
  public void swerveDrive(
    SwerveModuleState[] states) {
      this.frontLeft.updateState(states[0], driveState);
      this.frontRight.updateState(states[1], driveState);
      this.backLeft.updateState(states[2], driveState);
      this.backRight.updateState(states[3], driveState);
  }

  // returns the positions of all the swerve modules
  public SwerveModulePosition[] getPositions() {
    return new SwerveModulePosition[] {
      frontLeft.getPosition(), 
      frontRight.getPosition(), 
      backLeft.getPosition(), 
      backRight.getPosition()
    }; 
  }
  
  // returns the speeds and angles of all the swerve modules
  public SwerveModuleState[] getStates() {
    return new SwerveModuleState[] {
      frontLeft.getCurrentState(), 
      frontRight.getCurrentState(), 
      backLeft.getCurrentState(), 
      backRight.getCurrentState()
    }; 
  }



  // returns the speed of the robot 
  public ChassisSpeeds getChassisSpeeds() {
    return Constants.DrivetrainConstants.kDriveKinematics.toChassisSpeeds(getStates()); 
  }

  // returns the current odometry pose of the robot
  public Pose2d getPose() {
    return odometry.getEstimatedPosition(); 
  }

  // returns the direction the robot is facing in degrees from -180 to 180 degrees.
  public double getHeading() {
      return gyro.getRotation2d().getDegrees();
  }

  // zeros the current heading of the robot
  public void zeroHeading() {
    gyro.zeroYaw();
  }

  // gets the raw heading of the robot
  public double getYaw() {
      return -gyro.getYaw(); 
  }

  // Returns the rate at which the robot is turning in degrees per second.
  public double getTurnRate() {
      return -gyro.getRate();
  }

  // reset the current pose of the robot to a set pose
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(gyro.getRotation2d(), getPositions(), pose);
  }

  /**
   * set the drive state of the robot
   * @see DriveState
   */ 
  public void setDriveState(DriveState state) {
    this.driveState = state; 
  }
}
