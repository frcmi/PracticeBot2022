package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import static frc.robot.Constants.*;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */

  /*
   * Initializing solenoids
   * public Solenoid solenoidLeft1 = new Solenoid(PneumaticsModuleType.CTREPCM,
   * 0); //maybe need to change module type
   * public Solenoid solenoidLeft2 = new Solenoid(PneumaticsModuleType.CTREPCM,
   * 1);
   * public Solenoid solenoidRight1 = new Solenoid(PneumaticsModuleType.CTREPCM,
   * 2);
   * public Solenoid solenoidRight2 = new Solenoid(PneumaticsModuleType.CTREPCM,
   * 3);
   */

  // Drivetrain and motor initialization
  WPI_TalonFX rearLeft = new WPI_TalonFX(REAR_LEFT_MOTOR_ID);
  WPI_TalonFX frontLeft = new WPI_TalonFX(FRONT_LEFT_MOTOR_ID);
  WPI_TalonFX frontRight = new WPI_TalonFX(FRONT_RIGHT_MOTOR_ID);
  WPI_TalonFX rearRight = new WPI_TalonFX(REAR_RIGHT_MOTOR_ID);

  MotorControllerGroup left = new MotorControllerGroup(frontLeft, rearLeft);
  MotorControllerGroup right = new MotorControllerGroup(frontRight, rearRight);
  private DifferentialDrive difDrive = new DifferentialDrive(left, right);
  // autonomous navx initialization
  private AHRS ahrs = new AHRS();
  private final DifferentialDriveOdometry odometry;
  public Pose2d startingPose = new Pose2d();
  public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);

  public DriveTrain() {
    // rearLeft.configClosedloopRamp(.5);
    // rearRight.configClosedloopRamp(.5);
    // frontLeft.configClosedloopRamp(.5);
    // frontRight.configClosedloopRamp(.5);
    resetEncoders();
    odometry = new DifferentialDriveOdometry(ahrs.getRotation2d());
    left.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(ahrs.getRotation2d(),
        (rearLeft.getSelectedSensorPosition() + frontLeft.getSelectedSensorPosition()) / 2,
        (rearRight.getSelectedSensorPosition() + frontRight.getSelectedSensorPosition()) / 2);

  }

  // motors and driving
  public void drive(double l, double r) {
    difDrive.tankDrive(l, r);
  }

  public void stop() {
    difDrive.tankDrive(0, 0);
  }

  public double getLeftMotors() {
    return left.get();
  }

  public double getRightMotors() {
    return right.get();
  }

  public double getPitch() {
    return ahrs.getPitch();
  }

  public double getRoll() {
    return ahrs.getRoll();
  }

  public double getYaw() {
    return ahrs.getYaw();
  }

  // encoders
  public void setEncoders() { // need to set distance per pulse in constants and in here
    //falcon encoders maybe can't be set in pulse mode or something like that?
    rearLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    frontLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    rearRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    frontRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    
  }

  // Autonomous
  public Pose2d getPose() { // position related to how much moved (distance) and angular movement (rotation)
    return odometry.getPoseMeters();
  }

  public Pose2d getStartingPose() {
    return startingPose;
  }

  public void resetOdometry(Pose2d pose) { // resets position to input parameter pose
    resetEncoders();
    zeroHeading();
    startingPose = pose;
    odometry.resetPosition(pose, ahrs.getRotation2d());
  }

  public void resetEncoders() {
    rearLeft.setSelectedSensorPosition(0);
    frontLeft.setSelectedSensorPosition(0);
    rearRight.setSelectedSensorPosition(0);
    frontRight.setSelectedSensorPosition(0);
  }

  public void zeroHeading() { // resets gyro so angle diff is 0
    ahrs.reset();
  }

  public double getHeading() { // gets the offset in degrees
    return ahrs.getRotation2d().getDegrees();
  }

  public double getTurnRate() {
    return -ahrs.getRate();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        (rearLeft.getSelectedSensorVelocity() + frontLeft.getSelectedSensorVelocity()) / 2,
        (rearRight.getSelectedSensorVelocity() + frontRight.getSelectedSensorVelocity()) / 2);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) { // tank drive but with volts instead of perecent
                                                                    // output
    left.setVoltage(leftVolts);
    right.setVoltage(rightVolts);
    difDrive.feed();
  }

  public double getLeftDisplacement() {
    return (rearLeft.getSelectedSensorPosition() + frontLeft.getSelectedSensorPosition()) / 2;

  }

  public double getRightDisplacement() {
    return (rearRight.getSelectedSensorPosition() + frontLeft.getSelectedSensorPosition()) / 2;
  }
  public double getAverageEncoderDistance() {
    return (getLeftDisplacement() + getRightDisplacement()) / 2.0;
  }
}