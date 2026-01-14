package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  // Hardware Components
  private final SparkMax m_motor = new SparkMax(Constants.ShooterConstants.kMotorID, MotorType.kBrushless);
  private final SparkClosedLoopController m_pidController = m_motor.getClosedLoopController();
  private final RelativeEncoder m_encoder = m_motor.getEncoder();
  
  // Controller Reference
  private final CommandPS5Controller m_driverController;

  // Toggle Logic Variables
  private boolean m_currentButtonState = false;
  private boolean m_lastButtonState = false;
  private boolean m_isMotorRunning = false;

  /**
   * Constructor for ShooterSubsystem
   * @param controller The PS5 controller used to trigger the shooter
   */
  public ShooterSubsystem(CommandPS5Controller controller) {
    this.m_driverController = controller;
    
    // Motor Configuration
    com.revrobotics.spark.config.SparkMaxConfig config = new com.revrobotics.spark.config.SparkMaxConfig();
    config.smartCurrentLimit(40); // Sets current limit to 40 Amps
    
    // Apply configurations to the motor
    m_motor.configure(config, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // Read the Square button state from the PS5 controller
    m_currentButtonState = m_driverController.getHID().getSquareButton();

    // Check for "Rising Edge" (button just pressed)
    if (m_currentButtonState && !m_lastButtonState) {
        m_isMotorRunning = !m_isMotorRunning; // Flip the state
        
        if (m_isMotorRunning) {
            setSpeed(3000); // Start motor at 3000 RPM
        } else {
            stop(); // Turn off the motor
        }
    }
    
    // Save current state for the next loop
    m_lastButtonState = m_currentButtonState;
  }

  /**
   * Sets the shooter speed using PID velocity control
   * @param targetRPM The desired speed in RPM
   */
  public void setSpeed(double targetRPM) {
    m_pidController.setReference(targetRPM, ControlType.kVelocity);
  }

  /**
   * Stops the motor immediately
   */
  public void stop() {
    m_motor.set(0);
  }
}
