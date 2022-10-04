// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
 
package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//import edu.wpi.first.wpilibj.motorcontrol.MotorController; unused import, yellow annoys me :(
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
//import com.ctre.phoenix.motorcontrol.*; unused import, yellow annoys me :(
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.XboxController;

/**
 * This is a demo program showing the use of the DifferentialDrive class,
 * specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  double leftAxis = 0;
  double rightAxis = 0;
  
  // true = foward
  // false = backward
  boolean isForward = true;

  long time = 0;
  double multiplier = 0.3;
  boolean DualStickDrive = false;
  boolean XboxDualStickDrive = false;
  boolean OneStickDrive = false;
  boolean SliderDrive = false;
  boolean XboxOneStickDrive = true;
  boolean JoeyDrive = false;
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
    double leftAxis = 0;
    double rightAxis = 0;
    boolean rightInvert = false; //changes if t0 invert right moters
    boolean leftInvert = false;  //invert left moters

    if (XboxDualStickDrive) { //
      //leftMotors.setInverted(true);
      leftInvert = true;
      leftAxis = 0.95 * 0.8 * (xbox.getRawAxis(1));
      rightAxis = 0.8 * (xbox.getRawAxis(5));
      //m_myRobot.tankDrive(0.95 * 0.8 * (xbox.getRawAxis(1)), 0.8 * (xbox.getRawAxis(5)););
    }

    if (DualStickDrive) {
      leftInvert = true;
      leftAxis = 0.8 * (m_leftStick.getRawAxis(1));
      rightAxis = 0.8 * (m_rightStick.getRawAxis(5));
      //leftMotors.setInverted(true);
      //m_myRobot.tankDrive(0.8 * (m_leftStick.getRawAxis(1)), 0.8 * (m_rightStick.getRawAxis(5)));
    }

    if(XboxOneStickDrive) {
      if (Math.abs(xbox.getLeftY()) <= 0.2)  {
        leftAxis = 0.8 * (-(xbox.getLeftY()) + xbox.getLeftX());
        rightAxis = 0.8 * (xbox.getLeftY() + xbox.getLeftX());
        //m_myRobot.tankDrive(0.8 * (-(xbox.getLeftY()) + xbox.getLeftX()),
        //    0.8 * (xbox.getLeftY() + xbox.getLeftX()));
      } else {
        leftAxis = (0.8 * (-(xbox.getLeftY()) + 0.5 * xbox.getLeftX()));
        rightAxis = 0.8 * (xbox.getLeftY() + 0.5 * xbox.getLeftX());
        //m_myRobot.tankDrive((0.8 * (-(xbox.getLeftY()) + 0.5 * xbox.getLeftX())), 0.8 * (xbox.getLeftY() + 0.5 * xbox.getLeftX()));
      }
    }
    if (OneStickDrive) {
      if (m_leftStick.getY() >= -0.2 && m_leftStick.getY() <= 0.2) {
        leftAxis = 0.8 * (-(m_leftStick.getY()) + m_leftStick.getX());
        rightAxis = 0.8 * (m_leftStick.getY() + m_leftStick.getX());
        //m_myRobot.tankDrive(0.8 * (-(m_leftStick.getY()) + m_leftStick.getX()),
        //    0.8 * (m_leftStick.getY() + m_leftStick.getX()));
      } else {
        leftAxis = 0.8 * (-(m_leftStick.getY()) + 0.5 * m_leftStick.getX());
        rightAxis = 0.95 * (0.8 * (m_leftStick.getY() + 0.5 * m_leftStick.getX()));
        //m_myRobot.tankDrive(0.8 * (-(m_leftStick.getY()) + 0.5 * m_leftStick.getX()),
        //    0.95 * (0.8 * (m_leftStick.getY() + 0.5 * m_leftStick.getX())));
      }
      
      //    if (m_leftStick.getY() >= -0.2 && m_leftStick.getY() <= 0.2) {
      //   m_myRobot.tankDrive(0.8 * (-m_leftStick.getX()),
      //       0.8 * (m_leftStick.getX()));
      // } else {
      //   m_myRobot.tankDrive(0.8 * (-(m_leftStick.getY()) + 0.5 * m_leftStick.getX()),
      //       0.95 * (0.8 * (m_leftStick.getY() + 0.5 * m_leftStick.getX())));
      // }

    }
    if (SliderDrive) {
      leftInvert = true;
      //leftMotors.setInverted(true);
      if (m_leftStick.getRawAxis(3) > 0.75 || m_leftStick.getRawAxis(3) < -0.75) {
        leftAxis = m_leftStick.getRawAxis(3) * 0.8;
        // leftMotors.set(m_leftStick.getRawAxis(3) * 0.8);
      }
      if (m_rightStick.getRawAxis(3) > 0.75 || m_rightStick.getRawAxis(3) < -0.75) {
        rightAxis = m_rightStick.getRawAxis(3) * 0.8;
        //rightMotors.set(m_rightStick.getRawAxis(3) * 0.8);
      }
    }
    if(JoeyDrive) {
      rightInvert = true;
      //rightMotors.setInverted(true);

      if (xbox.getRawAxis(0) > 0){
        leftAxis = xbox.getRightTriggerAxis() - xbox.getLeftTriggerAxis() + (0.75 * xbox.getRawAxis(0));
        rightAxis = xbox.getRightTriggerAxis() - xbox.getLeftTriggerAxis();
        //m_myRobot.tankDrive(xbox.getRightTriggerAxis() - xbox.getLeftTriggerAxis() + (0.75 * xbox.getRawAxis(0)), 
        //                    xbox.getRightTriggerAxis() - xbox.getLeftTriggerAxis());
      } else if(xbox.getRightTriggerAxis() > 0 || xbox.getLeftTriggerAxis() > 0) {
        leftAxis = xbox.getRightTriggerAxis() - xbox.getLeftTriggerAxis();
        rightAxis = xbox.getRightTriggerAxis() - xbox.getLeftTriggerAxis() + (-0.75 * xbox.getRawAxis(0));
        //m_myRobot.tankDrive(xbox.getRightTriggerAxis() - xbox.getLeftTriggerAxis(), 
        //                    xbox.getRightTriggerAxis() - xbox.getLeftTriggerAxis() + (-0.75 * xbox.getRawAxis(0)));
      }
      else {
        leftAxis = xbox.getRawAxis(0);
        rightAxis = xbox.getRawAxis(0);
        //m_myRobot.tankDrive(xbox.getRawAxis(0), xbox.getRawAxis(0));
      }
    }
    /*
     * leftMotors.set(0.875 * 0.85);
     * rightMotors.setInverted(true);
     * rightMotors.set(0.85);
     */
  

    // If stopped: reset multiplier
    boolean currentBackward = leftAxis < 0 && rightAxis < 0; 
    if ((leftAxis == 0 && rightAxis == 0) || (leftAxis > 0 && rightAxis > 0 && !isForward) || (currentBackward && isForward)) {
      multiplier = 0.3;
    } else if (System.currentTimeMillis() - time > 0.5 * 1000 && multiplier < 0.8) {
      // Increase multiplier if continuing in direction
      multiplier += 0.1;
    }
    
    // temp multiplier
    double appliedMultiplier = multiplier;
    if (currentBackward) {
      appliedMultiplier /= 2; 
    }

    // set inverts
    leftMotors.setInverted(leftInvert);
    rightMotors.setInverted(rightInvert);

    // set motor speeds
    m_myRobot.tankDrive(appliedMultiplier * leftAxis, appliedMultiplier * rightAxis);
  }
}
