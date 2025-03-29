// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// March 25, 2025
// Eipp Test Code

package frc.robot;

/* SPARK MAX MOTOR packages and classes we need */
import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

/* WPILIB packages and classes we need */
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.cameraserver.CameraServer;
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
  private final int left_follower_deviceID = 3; // Motor 3 follows motor 1

  private final int right_deviceID = 2;
  private final int right_follower_deviceID = 4; // Motor 4 follows motor 2

  private final SparkMax m_coralmotor;
  private final int coral_deviceID = 5;

  private final SparkMax m_leftDrive;
  private final SparkMax m_leftFollower;
  private final SparkMax m_rightDrive;
  private final SparkMax m_rightFollower;

  private final DifferentialDrive m_robotDrive;
  private final XboxController m_controller;
  private final XboxController m_secondController;
  private final Timer m_timer;

  /** Called once at the beginning of the robot program. */
  public Robot() {

    //start camera
    CameraServer.startAutomaticCapture();
    // Initialize the SPARK MAX MAIN DRIVE MOTORS
    // using the proper CAN ID, set above for now, and the motor type
    m_leftDrive = new SparkMax(left_deviceID, MotorType.kBrushed);
    m_leftFollower = new SparkMax(left_follower_deviceID, MotorType.kBrushed);
    m_rightDrive = new SparkMax(right_deviceID, MotorType.kBrushed);
    m_rightFollower = new SparkMax(right_follower_deviceID, MotorType.kBrushed);
    
    configure_and_invert_motor(m_leftDrive, "no follow");
    configure_and_invert_motor(m_leftFollower, "follow left");
    
    configure_motor(m_rightDrive, "no follow");  
    configure_motor(m_rightFollower, "follow right");

    // Create a configuration for the follower using 2025 API from REV
    // and apply the configuration

    //SparkMaxConfig configLeft = new SparkMaxConfig();
    //configLeft.follow(m_leftDrive.getDeviceId()); // Set to follow the leader's device ID
    //m_leftFollower.configure(configLeft, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    //SparkMaxConfig configRight = new SparkMaxConfig();
    //configLeft.follow(m_rightDrive.getDeviceId()); // Set to follow the leader's device ID
    //m_leftFollower.configure(configRight, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // Configure followers
    // Motor 3 follows motor 1 (left side)
    // m_leftFollower.setFollow(m_leftDrive);

    // Alternative syntax if needed
    // m_leftFollower.setFollow(m_leftDrive, false); // false = don't invert
    // follower relative to leader

    // Motor 4 follows motor 2 (right side)
    // m_rightFollower.setFollow(m_rightDrive);
    // Alternative syntax if needed
    // m_rightFollower.setFollow(m_rightDrive, false); // false = don't invert
    // follower relative to leader

    m_coralmotor = new SparkMax(coral_deviceID, MotorType.kBrushed);
    configure_motor(m_coralmotor, "no follow");

    m_robotDrive = new DifferentialDrive(m_leftDrive::set, m_rightDrive::set);

    // Set maximum output to 100% (default is already 1.0)
    m_robotDrive.setMaxOutput(1.0);

    m_controller = new XboxController(0);
    m_secondController = new XboxController(1); // Add this line with port 1
    m_timer = new Timer();

    SendableRegistry.addChild(m_robotDrive, m_leftDrive);
    SendableRegistry.addChild(m_robotDrive, m_rightDrive);
    SendableRegistry.addChild(m_robotDrive, m_leftFollower);
    SendableRegistry.addChild(m_robotDrive, m_rightFollower);


    // debug code below
    SmartDashboard.putString("Aquaram", "READY!");
  }

  /**
   * Do whatever we need to configure our SPARK MAX motor
   * 
   * TODO: check all configurations and ensure that is what we want...
   * March 5
   * 
   * @param motor
   */
  private void configure_and_invert_motor(SparkMax motor, String follow) {
    // Create a configuration object to set the properties for the left motor
    SparkMaxConfig motor_config = new SparkMaxConfig();

    // Increase voltage compensation to ensure consistent performance
    motor_config.voltageCompensation(12.0);
    /*
     * The parameter nominalVoltage is the
     * "Nominal voltage to compensate output to".
     * This means the motor controller will adjust its output to maintain consistent
     * performance
     * regardless of battery voltage fluctuations. For example, if you set it to
     * 12.0 volts,
     * the motor will behave the same way whether your battery is at 13 volts or
     * has dropped to 11 volts
     */

    // Configure motor settings
    // NOTE: innverting direction of a speed controller if pass true to
    // inverted(true)
    // NOTE: This call has no effect if the controller is a follower.
    // NOTE: To invert a follower, see the follow() method.
    motor_config.inverted(true) // Set to true if you need to invert the motor
        .idleMode(SparkBaseConfig.IdleMode.kBrake) // Use brake mode
        .smartCurrentLimit(40) // Set current limit to 40 amps
        .openLoopRampRate(0.5); // Ramp rate of 0.5 seconds from 0 to full

    if (follow.equals("follow left")) {
      // ASSERT: motor passed in was m_leftFollower
      motor_config.follow(m_leftDrive.getDeviceId()); // Set to follow the leader's device ID
    } else if (follow.equals("follow right")) {
      // ASSERT: motor passed in was m_rightFollower
      motor_config.follow(m_rightDrive.getDeviceId()); // Set to follow the leader's device ID
    } else {
      ; // motor passed in is not a follower!
      // ASSERT: motor passed in was m_leftDrive or m_rightDrive
    }


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
  private void configure_motor(SparkMax motor, String follow) {
    // Create a configuration object to set the properties for the left motor
    SparkMaxConfig motor_config = new SparkMaxConfig();

    // Increase voltage compensation to ensure consistent performance
    motor_config.voltageCompensation(12.0);
    /*
     * The parameter nominalVoltage is the
     * "Nominal voltage to compensate output to".
     * This means the motor controller will adjust its output to maintain consistent
     * performance
     * regardless of battery voltage fluctuations. For example, if you set it to
     * 12.0 volts,
     * the motor will behave the same way whether your battery is at 13 volts or
     * has dropped to 11 volts
     */

    // Configure motor settings
    // NOTE: innverting direction of a speed controller if pass true to
    // inverted(true)
    // NOTE: This call has no effect if the controller is a follower.
    // NOTE: To invert a follower, see the follow() method.
    motor_config.inverted(false) // Set to true if you need to invert the motor
        .idleMode(SparkBaseConfig.IdleMode.kBrake) // Use brake mode
        .smartCurrentLimit(40) // Set current limit to 40 amps
        .openLoopRampRate(0.5); // Ramp rate of 0.5 seconds from 0 to full


    if (follow.equals("follow left")) {
      // ASSERT: motor passed in was m_leftFollower
      motor_config.follow(m_leftDrive.getDeviceId()); // Set to follow the leader's device ID
    } else if (follow.equals("follow right")) {
      // ASSERT: motor passed in was m_rightFollower
      motor_config.follow(m_rightDrive.getDeviceId()); // Set to follow the leader's device ID
    } else {
      ; // motor passed in is not a follower!
      // ASSERT: motor passed in was m_leftDrive or m_rightDrive
    }



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
    if (m_timer.get() < 1.90) {
      // Drive forwards half speed, make sure to turn input squaring off
      m_robotDrive.arcadeDrive(0.5, 0.0, false);

    } else {
        m_robotDrive.stopMotor(); // stop robot
        if (m_timer.get() < 3.00) {
          m_coralmotor.set(.6); // testing  
        } else {   
          m_coralmotor.set(0);
          m_robotDrive.stopMotor();
        }

     
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

    // Check if B button is pressed for emergency stop. aka: BREAK
    if (m_controller.getBButton()) {
      // Stop all drive motors
      m_robotDrive.stopMotor();
      // Stop coral motor
      m_coralmotor.set(0);
      
      // Optional: Display brake status on dashboard
      SmartDashboard.putBoolean("Emergency Brake Active", true);
      return; // Exit the method early to prevent other drive commands
    } else {
      SmartDashboard.putBoolean("Emergency Brake Active", false);
    }

    // TODO: test square inputs, this should be true for finer grained steering
    boolean squareInputs = false;

    // setting squareInputs to false will give you a linear response curve:
    // Use linear input response -- this will speed up robot, but reduce finer
    // control
    // m_robotDrive.arcadeDrive(-m_controller.getLeftY(), -m_controller.getRightX(),
    // !squareInputs);

    // control driving with our first controller
    m_robotDrive.arcadeDrive(-m_controller.getLeftY(), -m_controller.getRightX(), squareInputs);

    double speedMultiplier = 4.0;

    // TODO: test below if we want it
    // BOOST speed when A button is held
    if (m_controller.getAButton()) { 
      speedMultiplier =4.0; // 1.0 is Full speed
    } else {
      speedMultiplier = 4.0; // Normal speed
    }


    // TODO: Adjust accordingly
    double turnSpeedMultiplier = 4.0; // Reduce this value to make turning slower
    /*
      turnSpeedMultiplier (set to 0.7 or any value between 0.3-0.8 for slower turning)
      You can adjust the turnSpeedMultiplier value to fine-tune the turning sensitivity. 
      Lower values (like 0.5) will make turning much slower, while values closer to 1.0 will be more responsive
    */
    
    // Apply different multipliers to drive and turn inputs
    m_robotDrive.arcadeDrive(
        -m_controller.getLeftY() * speedMultiplier, 
        -m_controller.getRightX() * turnSpeedMultiplier, 
        squareInputs
    );


    // TODO: test below if we want it
    // Scale inputs by 1.5x (will be clamped to [-1.0, 1.0])
    // m_robotDrive.arcadeDrive(-m_controller.getLeftY() * speedMultiplier, -m_controller.getRightX() * speedMultiplier);

    // Control the coral motor with the second controller
    // You can use any axis you prefer - this example uses the left Y axis

    // TODO: SET CORAL MOTOR TO A SLOWER SPEED OR ADD A DEFAULT SPEED AND THE A
    // BOOST IF A IS PRESSED
    /* 
    double coralPower = -m_secondController.getLeftY();
    m_coralmotor.set(coralPower);
    // Optionally add to dashboard for debugging
    SmartDashboard.putNumber("Coral Motor Power", coralPower);
    */

    // For bidirectional control using both triggers
    double coralPower = m_controller.getRightTriggerAxis() - m_controller.getLeftTriggerAxis();
    // coralPower = coralPower * 0.6; // Adjust maximum power if needed
    // m_coralmotor.set(coralPower);
    if (m_controller.getAButton()) { 
      coralPower = coralPower * 1.0; // 1.0 is Full speed
    }
      else {
        coralPower = coralPower * 0.6;
      }
      m_coralmotor.set(coralPower);


    // m_coralmotor.set(coralPower);
    SmartDashboard.putNumber("Coral Motor Power", coralPower);

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

    showDebugValues("disabledPeriodic");
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

    showDebugValues("robotPeriodic");

  }

  private void showDebugValues(String caller) {

    SmartDashboard.putString("periodic caller", caller);

    // Monitoring for main left drive motor
    SmartDashboard.putNumber("m_leftDrive__Temperature", m_leftDrive.getMotorTemperature());
    SmartDashboard.putNumber("m_leftDrive__Current", m_leftDrive.getOutputCurrent());
    SmartDashboard.putNumber("m_leftDrive__Voltage", m_leftDrive.getBusVoltage() * m_leftDrive.getAppliedOutput());

    // Monitoring for main right drive motor
    SmartDashboard.putNumber("m_rightDrive_TEMP", m_rightDrive.getMotorTemperature());
    SmartDashboard.putNumber("m_rightDrive_CURRENT", m_rightDrive.getOutputCurrent());
    SmartDashboard.putNumber("m_rightDrive_VOLTAGE", m_rightDrive.getBusVoltage() * m_leftDrive.getAppliedOutput());

    // Monitoring for left follower motors
    SmartDashboard.putNumber("m_leftFollower_Temperature", m_leftFollower.getMotorTemperature());
    SmartDashboard.putNumber("m_leftFollower_Current", m_leftFollower.getOutputCurrent());
    SmartDashboard.putNumber("m_leftFollower_VOLTAGE", m_leftFollower.getBusVoltage() * m_leftDrive.getAppliedOutput());

    // Monitoring for left follower motors
    SmartDashboard.putNumber("m_rightFollower_Temperature", m_rightFollower.getMotorTemperature());
    SmartDashboard.putNumber("m_rightFollower_Current", m_rightFollower.getOutputCurrent());
    SmartDashboard.putNumber("m_rightDrive_VOLTAGE", m_rightFollower.getBusVoltage() * m_leftDrive.getAppliedOutput());

    // Add monitoring for coral motor
    SmartDashboard.putNumber("m_coralmotor_Temperature", m_coralmotor.getMotorTemperature());
    SmartDashboard.putNumber("m_coralmotor_Current", m_coralmotor.getOutputCurrent());
    SmartDashboard.putNumber("m_coralmotor_Voltage", m_coralmotor.getBusVoltage() * m_coralmotor.getAppliedOutput());

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

    // Check faults for all motors
    checkAndDisplayFaults(m_leftDrive, "Left Drive");
    checkAndDisplayFaults(m_rightDrive, "Right Drive");
    checkAndDisplayFaults(m_leftFollower, "Left Follower");
    checkAndDisplayFaults(m_rightFollower, "Right Follower");
    checkAndDisplayFaults(m_coralmotor, "Coral Motor");

  }

  private void checkAndDisplayFaults(SparkMax motor, String motorName) {
    Faults faults = motor.getFaults();
    if (faults != null) {
      StringBuilder faultString = new StringBuilder();

      // Check each fault flag using the actual fields from the 2025 API
      if (faults.other)
        faultString.append("Other, ");
      if (faults.motorType)
        faultString.append("Motor Type, ");
      if (faults.sensor)
        faultString.append("Sensor, ");
      if (faults.can)
        faultString.append("CAN, ");
      if (faults.temperature)
        faultString.append("Temperature, ");
      if (faults.gateDriver)
        faultString.append("Gate Driver, ");
      if (faults.escEeprom)
        faultString.append("ESC EEPROM, ");
      if (faults.firmware)
        faultString.append("Firmware, ");

      // Remove trailing comma and space if any faults were found
      if (faultString.length() > 0) {
        faultString.setLength(faultString.length() - 2);
      } else {
        faultString.append("None");
      }

      // show the debug string
      SmartDashboard.putString(motorName + " Faults", faultString.toString());

      // Display the raw fault bits for debugging
      SmartDashboard.putNumber(motorName + " Raw Fault Bits", faults.rawBits);
    }
  }

}
