// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This is a demo program showing the use of the DifferentialDrive class,
 * specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {

  private final DifferentialDrive m_robotDrive;
  private final Joystick m_leftStick;
  private final Joystick m_rightStick;
  private final int left_deviceID1 = 1; //when behind the robot: top left motor
  private final int right_deviceID1 = 2;//bottom right motor
  private final int left_deviceID2 = 3; //bottom left motor
  private final int right_deviceID2 = 4; //top right motor
  private final int coral_deviceID = 5;
  private final SparkMax m_leftDrive1;
  private final SparkMax m_leftDrive3;
  private final SparkMax m_rightDrive2;
  private final SparkMax m_rightDrive4;
  private final SparkMax m_coralmotor;
  private final XboxController m_controller;
  

  public final Timer m_timer;

  private double auto_count = 0;

  // private final SparkMax m_leftMotor = new SparkMax(0);
  // private final SparkMax m_rightMotor = new SparkMax(1);

  /** Called once at the beginning of the robot program. */
  public Robot() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    // m_rightMotor.setInverted(true); no need for this

    // m_robotDrive = new DifferentialDrive(m_leftMotor::set, m_rightMotor::set);

    m_leftDrive1 = new SparkMax(left_deviceID1, MotorType.kBrushed);
    configure_and_invert_motor(m_leftDrive1);
    m_leftDrive3 = new SparkMax(left_deviceID2, MotorType.kBrushed);
    configure_and_invert_motor(m_leftDrive3);

    // m_leftDrive1.addFollower(m_leftDrive3);

    m_rightDrive2 = new SparkMax(right_deviceID1, MotorType.kBrushed);
    configure_motor(m_rightDrive2);
    m_rightDrive4 = new SparkMax(right_deviceID2, MotorType.kBrushed);
    configure_motor(m_rightDrive4);

    m_coralmotor = new SparkMax(coral_deviceID, MotorType.kBrushed);
    configure_motor(m_coralmotor);

    m_robotDrive = new DifferentialDrive(m_leftDrive1::set, m_rightDrive2::set);
    m_controller = new XboxController(0);
    m_timer = new Timer();


    m_leftStick = new Joystick(0);
    m_rightStick = new Joystick(1);

    SendableRegistry.addChild(m_robotDrive, m_leftDrive1);
    SendableRegistry.addChild(m_robotDrive, m_leftDrive3);
    SendableRegistry.addChild(m_robotDrive, m_rightDrive2);
    SendableRegistry.addChild(m_robotDrive, m_rightDrive4);
  }

  private void configure_motor(SparkMax motor) {
    // Create a configuration object to set the properties for the left motor
    SparkMaxConfig motor_config = new SparkMaxConfig();

    // Configure motor settings
    // NOTE: innverting direction of a speed controller if pass true to
    // inverted(true)
    // NOTE: This call has no effect if the controller is a follower.
    // NOTE: To invert a follower, see the follow() method.
    motor_config.inverted(false) // Set to true if you need to invert the motor
        .idleMode(SparkBaseConfig.IdleMode.kBrake) // Use brake mode
        .smartCurrentLimit(40) // Set current limit to 40 amps
        .openLoopRampRate(0.5); // Ramp rate of 0.5 seconds from 0 to full

    // Apply the configuration to the SPARK MAX
    motor.configure(
        motor_config,
        ResetMode.kNoResetSafeParameters, // Don't reset existing parameters
        PersistMode.kNoPersistParameters // Don't save to flash memory

        /*
        * see configure_and_invert method for comments
        * on ResetMode and PersistMode
        */
    );
  }

  private void configure_and_invert_motor(SparkMax motor) {
    // Create a configuration object to set the properties for the left motor
    SparkMaxConfig motor_config = new SparkMaxConfig();

    // Configure motor settings
    // NOTE: innverting direction of a speed controller if pass true to
    // inverted(true)
    // NOTE: This call has no effect if the controller is a follower.
    // NOTE: To invert a follower, see the follow() method.
    motor_config.inverted(true) // Set to true if you need to invert the motor
        .idleMode(SparkBaseConfig.IdleMode.kBrake) // Use brake mode
        .smartCurrentLimit(40) // Set current limit to 40 amps
        .openLoopRampRate(0.5); // Ramp rate of 0.5 seconds from 0 to full

    // Apply the configuration to the SPARK MAX
    motor.configure(
        motor_config,
        ResetMode.kNoResetSafeParameters, // Don't reset existing parameters
        PersistMode.kNoPersistParameters // Don't save to flash memory

    /*
     * Understanding SparkBase.PersistMode.kNoPersistParameters
     * In the REVLib 2025 API, SparkBase.PersistMode.kNoPersistParameters
     * is used to control whether configuration settings are saved to the
     * SPARK MAX controller's non-volatile memory (flash memory).
     * 
     * When you use PersistMode.kNoPersistParameters (which appears to be
     * the updated name for what was previously called kDoNotPersist),
     * the configuration changes you make:
     * 
     * Are applied immediately to the controller's active settings
     * Are NOT saved to flash memory
     * Will be lost when power is cycled to the controller
     * 
     * When to Use kNoPersistParameters:
     * During development and testing:
     * When you're frequently changing settings
     * When you're testing different configurations
     * When you want to avoid excessive writes to flash memory
     * For runtime adjustments:
     * Settings that might change during operation
     * Temporary configurations
     * For safety-critical parameters:
     * When you want to ensure the robot always starts with known default values
     */

    );

  }

  @Override
  public void teleopPeriodic() {
    m_robotDrive.tankDrive(-m_leftStick.getY(), -m_rightStick.getY());
    if (m_controller.getAButton()) {
      // do what we want when A is pressed
    }
    SmartDashboard.putNumber("teleopPeriodic count: ", auto_count);
    auto_count++;
  }

  

  @Override
  public void autonomousInit() {
    // Code that runs once when entering autonomous mode

    m_timer.restart();

  }

  @Override
  public void autonomousPeriodic() {


    SmartDashboard.putNumber("autonomousPeriodic count: ", auto_count);
    auto_count++;

    // Drive for 2 seconds
    if (m_timer.get() < 2.0) {
      // Drive forwards half speed, make sure to turn input squaring off
      m_robotDrive.arcadeDrive(0.5, 0.0, false);
    } else {
      m_robotDrive.stopMotor(); // stop robot
    }
  }
  
  

}
