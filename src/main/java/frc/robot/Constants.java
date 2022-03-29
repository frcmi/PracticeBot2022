// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
//import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

/** Add your docs here. */
public class Constants {
    //Motor IDs
    public static final int FRONT_LEFT_MOTOR_ID = 3;
    public static final int REAR_LEFT_MOTOR_ID = 4;
    public static final int FRONT_RIGHT_MOTOR_ID = 2;
    public static final int REAR_RIGHT_MOTOR_ID = 1;

    //Drivetrain constants
    public static final int UNITS_PER_ROTATION = 2048; //not needed??

    public static final float GEAR_RATIO = (float) 11/120; //1 falcon rotation = 11/120 wheel rotation
    public static final double WHEEL_RADIUS_METERS = 0.0762;
    public static final double DISTANCE_PER_WHEEL_ROTATION = 2 * WHEEL_RADIUS_METERS * Math.PI;
    public static final double DISTANCE_PER_MOTOR_ROTATION = GEAR_RATIO * DISTANCE_PER_WHEEL_ROTATION;

    //Shooter constants
    
    //public static final PIDController SHOOTER_PID_AUTO = new PIDController(0, 0, 0); //adjust
    //public static final PIDController SHOOTER_PID_TELEOP = new PIDController(0.9, 0, 0); //adjust
    
    //PID setpoints
    public static final double AUTOSETPOINT = 0.75;
    public static final double TELEOPSETPOINT = 0.9;

    //PID Teleop Setpoints
    public static final double SHOOTER_PID_TELEOP_kP = 0.9;
    public static final double SHOOTER_PID_TELEOP_kI = 0;
    public static final double SHOOTER_PID_TELEOP_kD = 0;

    //PID Teleop Setpoints
    public static final double SHOOTER_PID_AUTO_kP = 0.9;
    public static final double SHOOTER_PID_AUTO_kI = 0;
    public static final double SHOOTER_PID_AUTO_kD = 0;

    //Feedforward constants
    public static final double kSVolts = 0.22;
    public static final double kVVoltSecondsPerRotation = 5;

    //Autnomous constants using preprogrammed names. CHANGE THEM!
    public static final double ksVolts = 0.22;
    public static final double kvVoltSecondsPerMeter = 1.98;
    public static final double kaVoltSecondsSquaredPerMeter = 0.2;
    public static final double kPDriveVel = 8.5;

    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;

    public static final double kTrackwidthMeters = 0.58; //dunno what this is
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);

    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
}
