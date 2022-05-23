// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Robot extends TimedRobot {
  private DifferentialDrive m_myRobot;
  private Joystick m_leftStick;
  private Joystick m_rightStick;

  private int LEFT_MOTOR_PORT = 0; //FIXME
  private int RIGHT_MOTOR_PORT = 0; //FIXME

  private double leftPower;
  private double rightPower;

  private final CANSparkMax m_leftMotor = new CANSparkMax(LEFT_MOTOR_PORT, MotorType.kBrushless);
  private final CANSparkMax m_rightMotor = new CANSparkMax(RIGHT_MOTOR_PORT, MotorType.kBrushless);

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
