/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMTalonFX;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {

  private static final int kFrontLeftChannel = 1;
  private static final int kFrontRightChannel = 2; 
  private static final int kRearLeftChannel = 3;
  private static final int kRearRightChannel = 4;
  private static final int rElevatorChannel = 0;
  private static final int rRollerChannel = 5;
  private static final int rLeftShooterChannel = 6;
  private static final int rRightShooterChannel = 7;
  private static final int rIntakeChannel = 7;
  
  private static final int kJoystickChannel = 0;
  private static final int rJoystickChannel = 1;

  private Timer m_timer; 

  private MecanumDrive m_robotDrive; 
  private SpeedController r_elevator;
  private SpeedController r_roller;
  private SpeedController r_lShooter;
  private SpeedController r_rShooter;
  private SpeedController r_intake;

  private Joystick m_stick;
  private XboxController r_stick;

  private double triggers;
  

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    //mecanum cim motors
    PWMVictorSPX frontLeft = new PWMVictorSPX(kFrontLeftChannel);
    PWMVictorSPX rearLeft = new PWMVictorSPX(kRearLeftChannel);
    PWMVictorSPX frontRight = new PWMVictorSPX(kFrontRightChannel);
    PWMVictorSPX rearRight = new PWMVictorSPX(kRearRightChannel);  

    //mecanum drive object
    m_robotDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);

    //Mission motor objects
    r_elevator = new PWMTalonFX(rElevatorChannel);
    r_roller = new PWMTalonFX(rRollerChannel);
    r_lShooter = new PWMTalonFX(rLeftShooterChannel);
    r_rShooter = new PWMTalonFX(rRightShooterChannel);
    r_intake = new PWMTalonFX(rIntakeChannel);

    //Joystick&controller objects
    m_stick = new Joystick(kJoystickChannel);
    r_stick = new XboxController(rJoystickChannel);    
   
    m_timer = new Timer();

    // Invert the left side motors.
    // You may need to change or remove this to match your robot.
    //frontLeft.setInverted(true);
    //rearLeft.setInverted(true);
    
    // Invert the left side shooter motor. 
    r_lShooter.setInverted(true);
 
  }

  /**
   * This function is run once each time the robot enters autonomous mode.
   */
  @Override
  public void autonomousInit() {

  	m_timer.reset();
    m_timer.start();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    // Drive for 2 seconds
    if (m_timer.get() < 2.0) {
      m_robotDrive.driveCartesian(0,0.5,0,0.0); // drive forwards half speed
    } else {
      m_robotDrive.stopMotor(); // stop robot
    }
  }

  /**
   * This function is called once each time the robot enters teleoperated mode.
   */
  @Override
  public void teleopInit() {
  }

  /**
   * This function is called periodically during teleoperated mode.
   */
  @Override
  public void teleopPeriodic() {

    triggers = (r_stick.getTriggerAxis(Hand.kRight))+(-1*r_stick.getTriggerAxis(Hand.kLeft));

    m_robotDrive.driveCartesian(m_stick.getX(), m_stick.getY(), m_stick.getZ(), 0.0);
    r_elevator.set(triggers);
    r_roller.set(r_stick.getY(Hand.kLeft));
    r_lShooter.set(r_stick.getY(Hand.kRight));
    r_rShooter.set(r_stick.getY(Hand.kRight));
  
    if (r_stick.getAButton()) {
      r_intake.set(1);
    }
    else {
      r_intake.set(0);
    }
    
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
