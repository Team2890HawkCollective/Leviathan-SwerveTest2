/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@SuppressWarnings("unused")
public class ClimberSubsystem extends SubsystemBase {

  /**
   * TalonFX Controllers - Falcon500 Winch Motors
   */
  private WPI_TalonFX leftTalonFX = new WPI_TalonFX(Constants.RIGHT_WINCH_TALON_FX_ID);
  // private WPI_TalonFX rightTalonFX = new
  // WPI_TalonFX(Constants.RIGHT_TALON_FX_ID);
  private WPI_CANCoder m_leftClimberEncoder = new WPI_CANCoder(Constants.RIGHT_WINCH_TALON_FX_ID);
  // private WPI_CANCoder m_rightClimbEncoder = new
  // WPI_CANCoder(Constants.RIGHT_CLIMBER_TALON_FX_ID);

  // private XboxController assistantDriverController = new
  // XboxController(Constants.XBOX_ASSISTANT_DRIVER_CONTROLLER_ID);

  // CANid=4 is config on the programming-board. 2022-03-09.
  private CANSparkMax sparkMax = new CANSparkMax(4, Constants.BRUSHLESS_MOTOR); // this works.

  /**
   * Xbox controller object used in the case the driver drives with an Xbox
   * controller
   */
  private XboxController driverController = new XboxController(Constants.XBOX_DRIVER_CONTROLLER_PORT_ID);

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
  public ClimberSubsystem() {
    System.out.println("DEVCHECK ClimberSubsystem constructor");
    // Sets left side of Spark Maxs inverted for proper functioning
    // leftTalonFX.setInverted(true);
    // leftBackSparkMax.setInverted(true);
  }

  private int counter = 0; // devTest 2022-03-09

  public void periodic() {
    counter++;

    double encPosition = m_leftClimberEncoder.getAbsolutePosition(); // this works.

    double position = leftTalonFX.getSensorCollection().getIntegratedSensorPosition();
    double absPosition = leftTalonFX.getSensorCollection().getIntegratedSensorAbsolutePosition();
    if ((counter % 100) == 0)
      System.out.println("leftTalonFX position=" + position
          + "  absPosition=" + absPosition + "  encPosition=-" + encPosition);
  }

  // Test running 2 motors on the programming-board.
  public void test() {
    leftTalonFX.set(0.1); // this works. Fixed the wiring.
    // sparkMax.set(0.1); // this works. 2022-03-09. 18:40.
  }
}
