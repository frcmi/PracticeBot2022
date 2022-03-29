// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import static frc.robot.Constants.*;

public class TrajectoryMaker extends CommandBase {
  DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
    new SimpleMotorFeedforward(kSVolts, kvVoltSecondsPerMeter), 
    kDriveKinematics, 10);
    TrajectoryConfig config =
    new TrajectoryConfig(kMaxSpeedMetersPerSecond,
                         kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(kDriveKinematics)
        // Apply the voltage constraint
        .addConstraint(autoVoltageConstraint);
  /** Creates a new TrajectoryAuto. */
  Trajectory trajectory;
  DriveTrain driveTrain;

  RamseteCommand ramseteCommand = new RamseteCommand(
    trajectory,
    driveTrain::getPose,
    new RamseteController(kRamseteB, kRamseteZeta),
    new SimpleMotorFeedforward(ksVolts,
                               kvVoltSecondsPerMeter,
                               kaVoltSecondsSquaredPerMeter),
    kDriveKinematics,
    driveTrain::getWheelSpeeds,
    new PIDController(kPDriveVel, 0, 0),
    new PIDController(kPDriveVel, 0, 0),
    // RamseteCommand passes volts to the callback
    driveTrain::tankDriveVolts,
    driveTrain
  );

  public TrajectoryMaker(Trajectory trajectory, DriveTrain driveTrain) {
    this.trajectory = trajectory;
    this.driveTrain = driveTrain;
    addRequirements(driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.resetOdometry(trajectory.getInitialPose());
    ramseteCommand.andThen(() -> driveTrain.tankDriveVolts(0, 0)).schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
