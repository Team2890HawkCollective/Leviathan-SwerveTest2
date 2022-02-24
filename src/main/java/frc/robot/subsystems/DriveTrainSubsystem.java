/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;

import frc.robot.Constants;

@SuppressWarnings("unused")
public class DriveTrainSubsystem extends SubsystemBase {
  public static final double kMaxSpeed = 3.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second
  
  /**
   * Spark Max Controllers - SwerveDrive Drive Motors (NEO Brushless)
   */
  private CANSparkMax leftFrontSparkMax = new CANSparkMax(Constants.LEFT_FRONT_SPARK_MAX_ID, Constants.BRUSHLESS_MOTOR);
  private CANSparkMax leftBackSparkMax = new CANSparkMax(Constants.LEFT_BACK_SPARK_MAX_ID, Constants.BRUSHLESS_MOTOR);
  private CANSparkMax rightFrontSparkMax = new CANSparkMax(Constants.RIGHT_FRONT_SPARK_MAX_ID,
      Constants.BRUSHLESS_MOTOR);
  private CANSparkMax rightBackSparkMax = new CANSparkMax(Constants.RIGHT_BACK_SPARK_MAX_ID, Constants.BRUSHLESS_MOTOR);

  // Drive Encoders - built-in to NEO connected to SparkMax controllers.
  RelativeEncoder leftFrontDriveEncoder = leftFrontSparkMax.getEncoder();
  RelativeEncoder leftBackDriveEncoder = leftBackSparkMax.getEncoder();
  RelativeEncoder rightFrontDriveEncoder = rightFrontSparkMax.getEncoder();
  RelativeEncoder rightBackDriveEncoder = rightBackSparkMax.getEncoder();

  /**
   * TalonSRX Controllers - SwerveDrive PG Steer Motors (PG71)
   */
  private WPI_TalonSRX leftFrontTalonSRX = new WPI_TalonSRX(Constants.LEFT_FRONT_TALON_SRX_ID);
  private WPI_TalonSRX leftBackTalonSRX = new WPI_TalonSRX(Constants.LEFT_BACK_TALON_SRX_ID);
  private WPI_TalonSRX rightFrontTalonSRX = new WPI_TalonSRX(Constants.RIGHT_FRONT_TALON_SRX_ID);
  private WPI_TalonSRX rightBackTalonSRX = new WPI_TalonSRX(Constants.RIGHT_BACK_TALON_SRX_ID);

  // Lamprey steer encoder (connected to TalonSRX)s- SwerveModule
  WPI_CANCoder leftFrontSteerEncoder = new WPI_CANCoder(Constants.LEFT_FRONT_TALON_SRX_ID);
  WPI_CANCoder leftBackSteerEncoder = new WPI_CANCoder(Constants.LEFT_BACK_TALON_SRX_ID);
  WPI_CANCoder rightFrontSteerEncoder = new WPI_CANCoder(Constants.RIGHT_FRONT_TALON_SRX_ID);
  WPI_CANCoder rightBackSteerEncoder = new WPI_CANCoder(Constants.RIGHT_BACK_TALON_SRX_ID);
  

  // private XboxController assistantDriverController = new
  // XboxController(Constants.XBOX_ASSISTANT_DRIVER_CONTROLLER_ID);

  /**
   * Xbox controller object used in the case the driver drives with an Xbox
   * controller
   */
  private XboxController driverController = new XboxController(Constants.XBOX_DRIVER_CONTROLLER_PORT_ID);

  /**
   * Swerve Drive Kinematics
   * The SwerveDriveKinematics class is a useful tool that converts between a
   * ChassisSpeeds object and several SwerveModuleState objects, which contains
   * velocities and angles for each swerve module of a swerve drive robot.
   */
  // Locations for the swerve drive modules relative to the robot center.
  Translation2d m_frontLeftLocation = new Translation2d(Constants.DISTANCE_FROM_CENTER,
      Constants.DISTANCE_FROM_CENTER);
  Translation2d m_frontRightLocation = new Translation2d(Constants.DISTANCE_FROM_CENTER,
      -Constants.DISTANCE_FROM_CENTER);
  Translation2d m_backLeftLocation = new Translation2d(-Constants.DISTANCE_FROM_CENTER,
      Constants.DISTANCE_FROM_CENTER);
  Translation2d m_backRightLocation = new Translation2d(-Constants.DISTANCE_FROM_CENTER,
      -Constants.DISTANCE_FROM_CENTER);

  // Creating my kinematics object using the module locations
  SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final AnalogGyro m_gyro = new AnalogGyro(Constants.GYRO_PORT);

  // Example chassis speeds: 1 meter per second forward, 3 meters
  // per second to the left, and rotation at 1.5 radians per second
  // counterclockwise.
  ChassisSpeeds speeds = new ChassisSpeeds(1.0, 3.0, 1.5);  // DevNote: Analyze and resolve this.

  // Convert to module states
  SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);

  // Front left module state
  SwerveModuleState frontLeft = moduleStates[0];

  // Front right module state
  SwerveModuleState frontRight = moduleStates[1];

  // Back left module state
  SwerveModuleState backLeft = moduleStates[2];

  // Back right module state
  SwerveModuleState backRight = moduleStates[3];

  /**
   * Joystick objects used in the case the driver drives with joysticks
   */
  private Joystick leftJoystick = new Joystick(Constants.DRIVER_JOYSTICK_Y_PORT_ID);
  private Joystick rightJoystick = new Joystick(Constants.DRIVER_JOYSTICK_X_PORT_ID);

  /**
   * Speeds used for arcade drive. Y for forwards and backwards. X for turning
   * left and right
   */
  private double yDriveSpeed = 0.0;
  private double xDriveSpeed = 0.0;

  /**
   * Speeds used for tank drive. Left for left side of bot. Right for right side
   * of bot
   */
  private double leftDriveSpeed = 0.0;
  private double rightDriveSpeed = 0.0;

  /**
   * Creates a new driveTrainSubsystem.
   */
  public DriveTrainSubsystem() {
    System.out.println("DEVCHECK DriveTrainSubsystem constructor");
    // Sets left side of Spark Maxs inverted for proper functioning
    leftFrontSparkMax.setInverted(true);
    leftBackSparkMax.setInverted(true);
  }

  /**
   * Method for using Xbox Controllers for arcade drive
   */
  public void xboxArcadeDrive() {
    // Sets forwards and backwards speed (y) to the y-axis of the left joystick.
    // Sets turning speed (x) tp x-axis of right joystick
    yDriveSpeed = driverController.getLeftY() * Constants.TELEOP_DRIVE_SPEED_MODIFIER;
    xDriveSpeed = driverController.getRightX() * Constants.TELEOP_DRIVE_SPEED_MODIFIER;

    // System.out.printf("DriveSpeed: LeftY=%s RightX=%s \n", xDriveSpeed,
    // yDriveSpeed);
    // Calls arcade drive method and sends speeds
    arcadeDrive(yDriveSpeed, xDriveSpeed);
  }

  /**
   * Method for using Joysticks for arcade drive
   */
  public void joystickArcadeDrive() {
    // Sets forwards and backwards speed (y) to the y-axis of the left joystick.
    // Sets turning speed (x) tp x-axis of right joystick
    yDriveSpeed = leftJoystick.getX() * Constants.TELEOP_DRIVE_SPEED_MODIFIER;
    xDriveSpeed = rightJoystick.getY() * Constants.TELEOP_DRIVE_SPEED_MODIFIER;

    // Calls arcade drive method and sends speeds
    arcadeDrive(-yDriveSpeed, -xDriveSpeed);
  }

  /**
   * Method for using an xbox controller for tank drive
   */
  public void xboxTankDrive() {
    // Sets left side of bot speed to the y-axis of the left joystick. Sets right
    // side of bot speed to y-axis of the right joystick
    leftDriveSpeed = driverController.getLeftY() * Constants.TELEOP_DRIVE_SPEED_MODIFIER;
    rightDriveSpeed = driverController.getRightY() * Constants.TELEOP_DRIVE_SPEED_MODIFIER;

    // Calls tank drive method and sends speeds
    tankDrive(leftDriveSpeed, rightDriveSpeed);
  }

  /**
   * Method for using Joystick controllers for tank drive
   */
  public void joystickTankDrive() {
    // Sets left side of bot speed to the y-axis of the left joystick. Sets right
    // side of bot speed to y-axis of the right joystick
    leftDriveSpeed = leftJoystick.getX() * Constants.TELEOP_DRIVE_SPEED_MODIFIER;
    rightDriveSpeed = rightJoystick.getX() * Constants.TELEOP_DRIVE_SPEED_MODIFIER;

    // Calls tank drive method and sends speeds
    tankDrive(leftDriveSpeed, rightDriveSpeed);
  }

  /**
   * Method that enables movement via arcade style drive
   */
  public void arcadeDrive(double ySpeed, double xSpeed) {
    // Assigns motor to forwards/backwards speed if no turning is detected
    drive(ySpeed, ySpeed);
    // If turning is detected, it will be added to one speed side and subtracted
    // from the other speed side to generate the effect of turning
    // whilst moving forwards/backwards at the same time
    if (xSpeed >= 0.05 || xSpeed <= -0.05) {
      drive(-xSpeed + ySpeed, xSpeed + ySpeed);
    }
  }

  /**
   * Method that enables movement via tank style drive
   */
  public void tankDrive(double leftJoySpeed, double rightJoySpeed) {
    drive(leftJoySpeed, rightJoySpeed);
  }

  /**
   * Assigns speeds to left and right controllers on bot
   */
  public void drive(double leftSpeed, double rightSpeed) {

    if (!Constants.isTargeting) {
      // Drive motors
      leftFrontSparkMax.set(leftSpeed);
      rightFrontSparkMax.set(rightSpeed);
      leftBackSparkMax.set(leftSpeed);
      rightBackSparkMax.set(rightSpeed);

      // Steer motors
      leftFrontTalonSRX.set(leftSpeed);
      rightFrontTalonSRX.set(rightSpeed); // DevNote: has an assembly mechanical problem. Need fix.
      leftBackTalonSRX.set(leftSpeed);
      rightBackTalonSRX.set(rightSpeed);
    }
  }
}
