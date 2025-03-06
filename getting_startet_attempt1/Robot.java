// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// March 5, 2025
// Mr. Billy Eipp
// Sample code to test and get robot running

package frc.robot;

/* SPARK MAX MOTOR packages and classes we need */
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

/* WPILIB packages and classes we need */
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/* 
 * The below code is from the sample code that generated this file 
 * and the objects are currently not being used 
 * 
 * import static edu.wpi.first.units.Units.Value;
 * import java.security.Key;
 * import org.ejml.equation.IntegerSequence.For;
 * import edu.wpi.first.units.measure.Current;
 * import edu.wpi.first.wpilibj.Encoder;
 * import edu.wpi.first.wpilibj.motorcontrol.Spark;
 * import edu.wpi.first.wpilibj2.command.Command;
 */

/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in the TimedRobot documentation. If you change the
 * name of this class or the package after creating this project, you must
 * also update the manifest file in the resource directory.
 */
public class Robot extends TimedRobot {

  private final int left_deviceID = 1;
  private final int right_deviceID = 2;
  private final SparkMax m_leftDrive;
  private final SparkMax m_rightDrive;
  private final DifferentialDrive m_robotDrive;
  private final XboxController m_controller;
  private final Timer m_timer;

  /** Called once at the beginning of the robot program. */
  public Robot() {

    // Initialize the SPARK MAX with the CAN ID and motor type
    // Replace "deviceID" value with your motor's CAN ID (typically 1-62)
    // TODO: need to validate left and right to see if correct here
    m_leftDrive = new SparkMax(left_deviceID, MotorType.kBrushed);
    configure_and_invert_motor(m_leftDrive);

    m_rightDrive = new SparkMax(right_deviceID, MotorType.kBrushed);
    configure_motor(m_rightDrive);

    m_robotDrive = new DifferentialDrive(m_leftDrive::set, m_rightDrive::set);
    m_controller = new XboxController(0);
    m_timer = new Timer();

    SendableRegistry.addChild(m_robotDrive, m_leftDrive);
    SendableRegistry.addChild(m_robotDrive, m_rightDrive);

    // debug code below
    SmartDashboard.putString("lets_go", "Aquamram Ready!");
  }

  /**
   * Do whatever we need to configure our SPARK MAX motor
   * 
   * TODO: check all configurations and ensure that is what we want...
   * March 5
   * 
   * @param motor
   */
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

  /**
   * Do whatever we need to configure our SPARK MAX motor
   * 
   * TODO: check all configurations and ensure that is what we want...
   * March 5
   * 
   * @param motor
   */
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

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    // Code that runs once when entering autonomous mode

    m_timer.restart();

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // Drive for 2 seconds
    if (m_timer.get() < 2.0) {
      // Drive forwards half speed, make sure to turn input squaring off
      m_robotDrive.arcadeDrive(0.5, 0.0, false);
    } else {
      m_robotDrive.stopMotor(); // stop robot
    }
  }

  /**
   * This function is called once each time the robot enters teleoperated mode.
   */
  @Override
  public void teleopInit() {
    // Code that runs once when entering teleop mode
  }

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    m_robotDrive.arcadeDrive(-m_controller.getLeftY(), -m_controller.getRightX());
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {
    // Code that runs once when entering test mode
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** this function is called when robot goes to enter disabled mode */
  @Override
  public void disabledInit() {
    // Code that runs once when entering disabled mode
  }

  @Override
  public void disabledPeriodic() {
    // This method is called periodically while the robot is in disabled mode

    // Most teams either:
    // 1. Leave this empty (but still override it to prevent the warning)
    // 2. Use it for diagnostic purposes like displaying sensor values

    // Example: If you want to update dashboard values while disabled
    SmartDashboard.putNumber("m_leftDrive__Temp", m_leftDrive.getMotorTemperature());
    SmartDashboard.putNumber("m_leftDrive__Temp", m_leftDrive.getMotorTemperature());

    // If you have nothing to do in disabled mode, just leave the method body empty
  }

  /*
   * Key points about robotPeriodic():
   * 
   * Keep it efficient: This method runs frequently, so avoid computationally
   * expensive operations
   * Use for robot-wide updates: Put code here that needs to run regardless of
   * robot mode
   * Dashboard updates: This is a good place to update SmartDashboard/Shuffleboard
   * values
   * Command scheduler: If using command-based programming, run the command
   * scheduler here
   * Sensor polling: Read sensors that need continuous monitoring
   * Avoid control code: Motor control code typically belongs in mode-specific
   * methods
   * For a SPARK MAX specifically, you might monitor:
   * 
   * Temperature
   * Current draw
   * Voltage
   * Encoder position/velocity (if using encoders)
   * Fault status
   * Remember that this method runs in ALL robot modes, including disabled, so
   * don't put any code here that would move the robot.
   */
  @Override
  public void robotPeriodic() {

    // TODO: this is only exmple code to get this started... feel free to remove :)
    // EIPP

    // 1. Update dashboard with sensor readings and robot state
    SmartDashboard.putNumber("m_leftDrive__Temperature", m_leftDrive.getMotorTemperature());
    SmartDashboard.putNumber("m_leftDrive__Current", m_leftDrive.getOutputCurrent());
    SmartDashboard.putNumber("m_leftDrive__Voltage", m_leftDrive.getBusVoltage() * m_leftDrive.getAppliedOutput());

    SmartDashboard.putNumber("m_rightDrive", m_rightDrive.getMotorTemperature());
    SmartDashboard.putNumber("m_rightDrive", m_rightDrive.getOutputCurrent());
    SmartDashboard.putNumber("m_rightDrive", m_rightDrive.getBusVoltage() * m_leftDrive.getAppliedOutput());

    // 2. Run subsystem periodic methods (if using command-based programming)
    // CommandScheduler.getInstance().run(); // If using WPILib command-based
    // framework

    // 3. Update odometry/position tracking (if applicable)
    // m_driveSubsystem.updateOdometry();

    // 4. Process vision data (if using vision)
    // processVisionData();

    // 5. Log data for later analysis
    // dataLogger.logData();

    // 6. Check for faults or error conditions
    Faults fault_m_rightDrive = m_rightDrive.getFaults();
    if (fault_m_rightDrive != null) {
      // Handle motor faults
      SmartDashboard.putString("m_rightDrive Faults", fault_m_rightDrive.toString());
    }

    Faults fault_m_leftDrive = m_leftDrive.getFaults();
    if (fault_m_leftDrive != null) {
      // Handle motor faults
      SmartDashboard.putString("fault_m_leftDrive Faults", fault_m_leftDrive.toString());
    }

  }

}
