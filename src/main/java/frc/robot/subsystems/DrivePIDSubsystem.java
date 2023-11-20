// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants;
import frc.robot.sensors.RomiGyro;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

public class DrivePIDSubsystem extends ProfiledPIDSubsystem {

  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  private final Spark m_leftMotor = new Spark(0);
  private final Spark m_rightMotor = new Spark(1);

  // The Romi has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder m_leftEncoder = new Encoder(4, 5);
  private final Encoder m_rightEncoder = new Encoder(6, 7);

/*   private ArmFeedforward m_feedforward =
  new ArmFeedforward(
      Constants.kSVolts, Constants.kGVolts,
      Constants.kVVoltSecondPerRad, Constants.kAVoltSecondSquaredPerRad);  */

  private SimpleMotorFeedforward m_feedforward =
  new SimpleMotorFeedforward(
      Constants.kSVolts, Constants.kVVoltSecondPerMeter, Constants.kAVoltSecondSquaredPerMeter); 

  // Set up the RomiGyro
  private final RomiGyro m_gyro = new RomiGyro();

  // Set up the BuiltInAccelerometer
  private final BuiltInAccelerometer m_accelerometer = new BuiltInAccelerometer();

  private double m_goalposition;

  // Used to put telemetry data and tuning parameters onto Shuffleboard
  GenericEntry m_avgDistanceEntry, m_speedEntry, m_goalEntry;
  GenericEntry m_setPosEntry, m_setVelEntry, m_PIDoutputEntry, m_feedforwardEntry;
  GenericEntry m_tuneP, m_tuneD, m_tuneI, m_tuneV, m_tuneA;
  GenericEntry m_tuneFS, m_tuneFG, m_tuneFV, m_tuneFA;

  private static NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private static NetworkTable m_table;
   
  /** Creates a new Drivetrain. */
  public DrivePIDSubsystem() {
    super(
        new ProfiledPIDController(
            Constants.kPDriveProfiled,
            Constants.kIDriveProfiled,
            Constants.kDDriveProfiled,
            new TrapezoidProfile.Constraints(
                Constants.kMaxSpeedMetersPerSecond,
                Constants.kMaxAccelMetersPerSecondSquared)),
        0);

    // Invert right side of the drivetrain so that positive voltages
    // result in both sides moving forward. Initialize to 0.
    m_rightMotor.setInverted(true);
    
    m_leftMotor.set(0.0);
    m_rightMotor.set(0.0);

    // Use meters as unit for encoder distances
    m_leftEncoder.setDistancePerPulse(2 * Math.PI * Constants.kWheelRadiusMeters / Constants.kEncoderResolution);
    m_rightEncoder.setDistancePerPulse(2 * Math.PI * Constants.kWheelRadiusMeters / Constants.kEncoderResolution);
   
    resetPosition();
    m_gyro.reset();

    // Start arm at rest in neutral position
    setGoal(Constants.kStartPosition);

    setupShuffleboard();

  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Calculate the feedforward from the setpoint
    double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
    // Add the feedforward to the PID output to get the motor output
    m_leftMotor.set(output + feedforward);
    m_rightMotor.set(output + feedforward);

    m_setPosEntry.setDouble(setpoint.position);
    m_setVelEntry.setDouble(setpoint.velocity);
    m_PIDoutputEntry.setDouble(output);
    m_feedforwardEntry.setDouble(feedforward);

  }

  @Override
  /** Enables the PID control. Resets the controller. */
  public void enable() {

    // Override PID and Feedforward parameters from Shuffleboard
    if (Constants.enablePIDTune) {
      DataLogManager.log("Update parameters");

      m_table = inst.getTable("Shuffleboard/PID Tuning");
      
      m_controller.setP(m_table.getEntry("kP").getDouble(Constants.kPDriveProfiled));
      m_controller.setI(m_table.getEntry("kI").getDouble(Constants.kIDriveProfiled));
      m_controller.setD(m_table.getEntry("kD").getDouble(Constants.kDDriveProfiled));
      double newVmax = m_table.getEntry("Vmax").getDouble(Constants.kMaxSpeedMetersPerSecond);
      double newAmax = m_table.getEntry("Amax").getDouble(Constants.kMaxAccelMetersPerSecondSquared);
      m_controller.setConstraints(new TrapezoidProfile.Constraints(newVmax, newAmax));

      double kS = m_table.getEntry("kS").getDouble(Constants.kSVolts);
      double kV = m_table.getEntry("kV").getDouble(Constants.kVVoltSecondPerMeter);
      double kA = m_table.getEntry("kA").getDouble(Constants.kAVoltSecondSquaredPerMeter);
      // double kG = m_table.getEntry("kG").getDouble(Constants.kGVolts);

      m_feedforward = new SimpleMotorFeedforward(kS, kV, kA);

    }

    m_enabled = true;
    m_controller.reset(getMeasurement());

    DataLogManager.log("PID control enabled");

  }
  
  /** Disables the PID control. Sets output to zero. */
  public void disable() {
    m_enabled = false;
    useOutput(0, new State());

    DataLogManager.log("PID control disabled");

  }

  @Override
  // Position for PID measurement
  public double getMeasurement() {
    return getAverageDistanceMeters();

  }

  // Speed (Meters/sec)
  public double getVelocity() {
    return (m_leftEncoder.getRate() + m_rightEncoder.getRate())/2;

  }

  // Reset the encoders to zero. Should only be used when arm is in neutral position.
  public void resetPosition() {
    // Arm position for PID measurement
    m_leftEncoder.reset() ;
    m_rightEncoder.reset() ;
  
  }

  // Calculate increased  goal limited to allowed range
  public double increasedGoal() {
    double newGoal = m_controller.getGoal().position + Constants.kPosIncrement;
    return MathUtil.clamp(newGoal, Constants.kminPosition, Constants.kmaxPosition);
  }

  // Calculate decreased  goal limited to allowed range
  public double decreasedGoal() {
    double newGoal =  m_controller.getGoal().position - Constants.kPosIncrement;
    return MathUtil.clamp(newGoal, Constants.kminPosition, Constants.kmaxPosition);

  }

  @Override
  public void periodic() {
    if (m_enabled) {
      useOutput(m_controller.calculate(getMeasurement()), m_controller.getSetpoint());
    }

    updateShuffleboard();

  }

  /**
   * Sets the goal state for the subsystem. Goal velocity assumed to be zero.
   *
   * @param goal The goal position for the subsystem's motion profile.
   */
  public void setGoal(double goal) {
    setGoal(new TrapezoidProfile.State(goal, 0));
    m_goalposition = goal;

  }

  //-----------------------------
  // Standard Romi methods
    
  public int getLeftEncoderCount() {
    return m_leftEncoder.get();
  }

  public int getRightEncoderCount() {
    return m_rightEncoder.get();
  }

  public double getLeftDistanceMeters() {
    return m_leftEncoder.getDistance();
  }

  public double getRightDistanceMeters() {
    return m_rightEncoder.getDistance();
  }

  public double getAverageDistanceMeters() {
    return (getLeftDistanceMeters() + getRightDistanceMeters()) / 2.0;
  }

  /**
   * The acceleration in the X-axis.
   *
   * @return The acceleration of the Romi along the X-axis in Gs
   */
  public double getAccelX() {
    return m_accelerometer.getX();
  }

  /**
   * The acceleration in the Y-axis.
   *
   * @return The acceleration of the Romi along the Y-axis in Gs
   */
  public double getAccelY() {
    return m_accelerometer.getY();
  }

  /**
   * The acceleration in the Z-axis.
   *
   * @return The acceleration of the Romi along the Z-axis in Gs
   */
  public double getAccelZ() {
    return m_accelerometer.getZ();
  }

  /**
   * Current angle of the Romi around the X-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleX() {
    return m_gyro.getAngleX();
  }

  /**
   * Current angle of the Romi around the Y-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleY() {
    return m_gyro.getAngleY();
  }

  /**
   * Current angle of the Romi around the Z-axis (wraps at +/- 180).
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleZ() {
    return -Math.IEEEremainder(m_gyro.getAngleZ(), 360);
  }

  /**
   * Current angle of the Romi around the Z-axis (continuous at +/- 180).
   *
   * @return The current angle of the Romi in degrees
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Rate of turn in degrees-per-second around the Z-axis.
   *
   * @return rate of turn in degrees-per-second
   */
  public double getGyroRateZ() {
    return -m_gyro.getRateZ();
  }

  // Reset the gyro. 
  public void resetGyro() {
    m_gyro.reset();
  }

  private void setupShuffleboard() {

    //SmartDashboard.putData(m_diffDrive);

    // Create a tab for the distance PID tuning if enabled
    if (Constants.enablePIDTune) {
      DataLogManager.log("Setup PID shuffleboard");

      ShuffleboardTab m_tuneTab = Shuffleboard.getTab("PID Tuning");

      // Display distance, velocity and current goal position
      m_avgDistanceEntry = m_tuneTab.add("Distance", getAverageDistanceMeters())
          .withWidget(BuiltInWidgets.kGraph)      
          .withSize(4,3)
          .withPosition(5, 0)
          .getEntry();

      m_speedEntry = m_tuneTab.add("Speed", (m_leftEncoder.getRate() + m_rightEncoder.getRate())/2)
          .withWidget(BuiltInWidgets.kGraph)      
          .withSize(4,3)
          .withPosition(1, 0)
          .getEntry();

      m_goalEntry = m_tuneTab.add("Goal", 0.0)
          .withPosition(5, 3)
          .getEntry();          

      m_setPosEntry = m_tuneTab.add("Set Pt Pos", 0.0)
          .withPosition(5, 4)
          .getEntry();
          
      m_setVelEntry = m_tuneTab.add("Set Pt Vel", 0.0)
          .withPosition(6, 4)
          .getEntry(); 

      m_PIDoutputEntry = m_tuneTab.add("PID Out", 0.0)
          .withPosition(7, 4)
          .getEntry(); 

      m_feedforwardEntry = m_tuneTab.add("Feedforward", 0.0)
          .withPosition(8, 4)
          .getEntry(); 

      // Add PID and Feedforward tuning parameters 
      m_tuneP = m_tuneTab.add("kP", Constants.kPDriveProfiled)
      .withPosition(0, 0)
      .getEntry();

      m_tuneI = m_tuneTab.add("kI", Constants.kIDriveProfiled)
      .withPosition(0, 1)
      .getEntry();

      m_tuneD = m_tuneTab.add("kD", Constants.kDDriveProfiled)
      .withPosition(0, 2)
      .getEntry();

      m_tuneV = m_tuneTab.add("Vmx", Constants.kMaxSpeedMetersPerSecond)
      .withPosition(0, 3)
      .getEntry();

      m_tuneA = m_tuneTab.add("Amx", Constants.kMaxAccelMetersPerSecondSquared)
      .withPosition(0, 4)
      .getEntry();

      m_tuneFS = m_tuneTab.add("kS", Constants.kSVolts)
      .withPosition(0, 5)
      .getEntry(); 

      m_tuneFV = m_tuneTab.add("kV", Constants.kVVoltSecondPerMeter)
      .withPosition(1, 5)
      .getEntry();

      m_tuneFA = m_tuneTab.add("kA", Constants.kAVoltSecondSquaredPerMeter)
      .withPosition(2, 5)
      .getEntry();
      
      // m_tuneFG = m_tuneTab.add("kG", Constants.kGVolts)
      // .withPosition(3, 5)
      // .getEntry();
     
    }

        
    // Create a tab for the Odometry and Field
    /*     ShuffleboardTab m_fieldTab = Shuffleboard.getTab("Field");
    m_fieldTab
        .add("Field", m_field);
    SmartDashboard.putData(m_field);

    ShuffleboardLayout commands = m_fieldTab.getLayout("Commands",BuiltInLayouts.kList);
    commands.add(new InstantCommand (() ->  resetOdometry())); */

  } 

  public void updateShuffleboard() {

    SmartDashboard.putNumber("Z Angle", getGyroAngleZ()); 
    SmartDashboard.putNumber("Z Rate", getGyroRateZ()); 

    SmartDashboard.putNumber("Left Rate", m_leftEncoder.getRate());
    SmartDashboard.putNumber("Right Rate", m_rightEncoder.getRate());
    SmartDashboard.putNumber("Average Rate", (m_leftEncoder.getRate() + m_rightEncoder.getRate())/2);

    // Update tuning debug if enabled
    if (Constants.enablePIDTune) {
      // DataLogManager.log("Update PID Tab");

      m_avgDistanceEntry.setDouble(getAverageDistanceMeters());
      m_speedEntry.setDouble((m_leftEncoder.getRate() + m_rightEncoder.getRate())/2);
      m_goalEntry.setDouble(m_goalposition);

    }

  }

}

  
