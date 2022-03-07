// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;

/**
 * This is a demo program showing the use of the DifferentialDrive class,
 * specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  
  WPI_VictorSPX m_rearRight = new WPI_VictorSPX(1);
  WPI_VictorSPX m_frontRight = new WPI_VictorSPX(2);
  WPI_VictorSPX m_frontLeft = new WPI_VictorSPX(3);
  WPI_VictorSPX m_rearLeft = new WPI_VictorSPX(4);
  MotorControllerGroup leftMotors = new MotorControllerGroup(m_frontLeft, m_rearLeft);
  MotorControllerGroup rightMotors = new MotorControllerGroup(m_frontRight, m_rearRight);

  //Joystick m_leftStick = new Joystick(0);
  Joystick m_leftStick = new Joystick(1);

  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    
  }

  @Override
  public void teleopPeriodic() {
    leftMotors.set(-(m_leftStick.getY()) + m_leftStick.getX());
    rightMotors.set(m_leftStick.getY() + m_leftStick.getX());
  }
}
