// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

public class Robot extends TimedRobot {
  private DifferentialDrive m_myRobot;
  private Joystick m_leftStick;
  private Joystick m_rightStick;

  private int LEFT_MOTOR_PORT = 0; //FIXME
  private int RIGHT_MOTOR_PORT = 1; //FIXME

  private double leftPower;
  private double rightPower;

  private final MotorController m_leftMotor = new PWMSparkMax(LEFT_MOTOR_PORT);
  private final MotorController m_rightMotor = new PWMSparkMax(RIGHT_MOTOR_PORT);

  @Override
  public void robotInit() {
    m_rightMotor.setInverted(true);

    m_myRobot = new DifferentialDrive(m_leftMotor, m_rightMotor);
    m_leftStick = new Joystick(0);
    m_rightStick = new Joystick(1);
  }

  @Override
  public void teleopPeriodic() {
    leftPower = m_leftStick.getY() + m_rightStick.getX();
    rightPower = m_leftStick.getY() - m_rightStick.getX();

    m_myRobot.tankDrive(leftPower, rightPower);
  }
}
