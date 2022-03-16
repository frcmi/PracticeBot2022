// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;

/**
 * This is a demo program showing the use of the DifferentialDrive class,
 * specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  SendableChooser driveChooser = new SendableChooser<Double>();

  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable driveTable = inst.getTable("drivetable");
  NetworkTableEntry driveEntry = driveTable.getEntry("driveentry");

  private DifferentialDrive m_myRobot;
  WPI_VictorSPX m_rearRight = new WPI_VictorSPX(1);
  WPI_VictorSPX m_frontRight = new WPI_VictorSPX(2);
  WPI_VictorSPX m_frontLeft = new WPI_VictorSPX(3);
  WPI_VictorSPX m_rearLeft = new WPI_VictorSPX(4);
  MotorControllerGroup leftMotors = new MotorControllerGroup(m_frontLeft, m_rearLeft);
  MotorControllerGroup rightMotors = new MotorControllerGroup(m_frontRight, m_rearRight);

  Joystick m_leftStick = new Joystick(0);
  Joystick m_rightStick = new Joystick(1);
  XboxController xbox = new XboxController(0);

  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.

    m_myRobot = new DifferentialDrive(leftMotors, rightMotors);
    driveChooser.addOption("One Stick", 1.0);
    driveChooser.addOption("Dual Stick", 2.0);
    driveChooser.addOption("Xbox", 3.0);
    driveChooser.addOption("Xbox One Stick", 4.0);

    driveEntry = Shuffleboard.getTab("Drive Mode")
        .addPersistent("Mode", driveChooser)
        .withWidget(BuiltInWidgets.kComboBoxChooser)
        .getEntry();

  }

  @Override
  public void teleopPeriodic() {
    double driveMode = driveEntry.getDouble(1.0);

    if (driveMode == 3.0) {
      leftMotors.setInverted(true);
      m_myRobot.tankDrive(0.95 * 0.8 * (xbox.getRawAxis(1)), 0.8 * (xbox.getRawAxis(5)));
    } else if (driveMode == 2.0) {
      leftMotors.setInverted(true);
      m_myRobot.tankDrive(0.8 * (m_leftStick.getRawAxis(1)), 0.8 * (m_rightStick.getRawAxis(1)));
    } else if (driveMode == 1.0) {
      if (m_leftStick.getY() >= -0.2 && m_leftStick.getY() <= 0.2) {
        m_myRobot.tankDrive(0.8 * (-(m_leftStick.getY()) + m_leftStick.getX()),
            0.8 * (m_leftStick.getY() + m_leftStick.getX()));
      } else {
        m_myRobot.tankDrive(0.8 * (-(m_leftStick.getY()) + 0.5 * m_leftStick.getX()),
            0.95 * (0.8 * (m_leftStick.getY() + 0.5 * m_leftStick.getX())));
      }
    } else if (driveMode == 4.0) {
      leftMotors.setInverted(true);
      m_myRobot.tankDrive(
          xbox.getRightTriggerAxis() * ((0.95 * 0.8 * xbox.getRawAxis(1)) + (0.95 * 0.8 * xbox.getRawAxis(2))),
          xbox.getRightTriggerAxis() * ((0.95 * 0.8 * xbox.getRawAxis(1)) - (0.95 * 0.8 * xbox.getRawAxis(2))));
    }
  }
}
