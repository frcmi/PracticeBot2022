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
import edu.wpi.first.wpilibj.XboxController;

/**
 * This is a demo program showing the use of the DifferentialDrive class,
 * specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  boolean DualStickDrive = false;
  boolean XboxDrive = false;
  boolean OneStickDrive = false;
  boolean SliderDrive = false;
  boolean XboxOneStick = true;

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

  }

  @Override
  public void teleopPeriodic() {

    if (XboxDrive) {
      leftMotors.setInverted(true);
      m_myRobot.tankDrive(0.95 * 0.8 * (xbox.getRawAxis(1)), 0.8 * (xbox.getRawAxis(5)));
    }

    if (XboxOneStick) {
      rightMotors.setInverted(true);

      while(xbox.getRawButton(5)){
        //System.out.println("slide");
        if (xbox.getRawAxis(0) > 0){
          m_myRobot.tankDrive(xbox.getRightTriggerAxis(), 
                              (-(xbox.getRightTriggerAxis())));
          
        } else if (xbox.getRawAxis(0) < 0) {
          m_myRobot.tankDrive((-(xbox.getRightTriggerAxis())), 
                              xbox.getRightTriggerAxis()); 
        }
      }

      if (xbox.getRawAxis(0) > 0){
        m_myRobot.tankDrive(xbox.getRightTriggerAxis() - xbox.getLeftTriggerAxis() + (0.75 * xbox.getRawAxis(0)), 
                            xbox.getRightTriggerAxis() - xbox.getLeftTriggerAxis());
      } else {
        m_myRobot.tankDrive(xbox.getRightTriggerAxis() - xbox.getLeftTriggerAxis(), 
                            xbox.getRightTriggerAxis() - xbox.getLeftTriggerAxis() + (-0.75 * xbox.getRawAxis(0)));
      }

      
    }

    if (DualStickDrive) {
      leftMotors.setInverted(true);
      m_myRobot.tankDrive(0.8 * (m_leftStick.getRawAxis(1)), 0.8 * (m_rightStick.getRawAxis(1)));
    }

    if (OneStickDrive) {
      if (m_leftStick.getY() >= -0.2 && m_leftStick.getY() <= 0.2) {
        m_myRobot.tankDrive(0.8 * (-(m_leftStick.getY()) + m_leftStick.getX()),
            0.8 * (m_leftStick.getY() + m_leftStick.getX()));
      } else {
        m_myRobot.tankDrive(0.8 * (-(m_leftStick.getY()) + 0.5 * m_leftStick.getX()),
            0.95 * (0.8 * (m_leftStick.getY() + 0.5 * m_leftStick.getX())));
      }

    }

    if (SliderDrive) {
      leftMotors.setInverted(true);
      if (m_leftStick.getRawAxis(3) > 0.75 || m_leftStick.getRawAxis(3) < -0.75) {
        leftMotors.set(m_leftStick.getRawAxis(3) * 0.8);
      }
      if (m_rightStick.getRawAxis(3) > 0.75 || m_rightStick.getRawAxis(3) < -0.75) {
        rightMotors.set(m_rightStick.getRawAxis(3) * 0.8);
      }
    }

    /*
     * leftMotors.set(0.875 * 0.85);
     * rightMotors.setInverted(true);
     * rightMotors.set(0.85);
     */
  }
}
