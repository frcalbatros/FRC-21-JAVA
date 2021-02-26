/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMTalonFX;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {

  private static final int mFrontRightChannel = 1; 
  private static final int mFrontLeftChannel = 2;
  private static final int mRearRightChannel = 3;
  private static final int mRearLeftChannel = 4;
  
  private static final int rElevatorChannel = 0;
  private static final int rRollerChannel = 5;
  private static final int rRightShooterChannel = 6;
  private static final int rLeftShooterChannel = 7;
  private static final int rIntakeChannel = 8;
  
  private static final int mJoystickChannel = 1; //driving joystick
  private static final int rJoystickChannel = 0; //Mission controller

  // gyro variables do not change
  private static final double kAngleSetpoint = 0.0;
	private static final double kP = 0.005; // propotional turning constant
  private static final SPI.Port kGyroPort = SPI.Port.kOnboardCS0;

  // Buttons
  private static final int fwd_button = 1; // joystick buton to lock y axis
  private static final int ltr_button = 2; // joystick buton to lock x axis
  private static final int gyro_button = 3; // assign button ID
  private static final int inc_button = 6;
  private static final int dec_button = 4;

  
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

  private double x_coef;
  private double y_coef;
  private double t_coef;
  private double ag_coef;
  

  

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    //mecanum cim motors
    PWMVictorSPX frontLeft = new PWMVictorSPX(mFrontLeftChannel);
    PWMVictorSPX rearLeft = new PWMVictorSPX(mRearLeftChannel);
    PWMVictorSPX frontRight = new PWMVictorSPX(mFrontRightChannel);
    PWMVictorSPX rearRight = new PWMVictorSPX(mRearRightChannel);

    ADXRS450_Gyro m_gyro = new ADXRS450_Gyro(kGyroPort);

    //mecanum drive object
    m_robotDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);

    //Mission motor objects
    r_elevator = new PWMTalonFX(rElevatorChannel);
    r_roller = new PWMTalonFX(rRollerChannel);
    r_lShooter = new PWMTalonFX(rLeftShooterChannel);
    r_rShooter = new PWMTalonFX(rRightShooterChannel);
    r_intake = new PWMTalonFX(rIntakeChannel);

    //Joystick&controller objects
    m_stick = new Joystick(mJoystickChannel);
    r_stick = new XboxController(rJoystickChannel);    
   
    m_timer = new Timer();


    // Invert the left side motors.
    // You may need to change or remove this to match your robot.
    //frontLeft.setInverted(true);
    //rearLeft.setInverted(true);
    
    // Invert the left side shooter motor. 
    r_rShooter.setInverted(true);

    //System.out.println("hello");

    // calibrate gyro
    m_gyro.calibrate();
 
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
    if (m_timer.get() < 1.0) {
      m_robotDrive.driveCartesian(0,0.3,0,0.0); // drive forwards half speed
    }else if (m_timer.get() > 1.0 && m_timer.get() < 2.0 ) {
      m_robotDrive.stopMotor();
    }else if (m_timer.get() > 3.0 && m_timer.get() < 4.0 ){
      m_robotDrive.driveCartesian(0.3,0,0,0.0); // drive forwards half speed
    }else if (m_timer.get() > 5.0 && m_timer.get() < 6.0 ) {
      m_robotDrive.stopMotor();
    }else if (m_timer.get() > 7.0 && m_timer.get() < 8.0 ) {
      m_robotDrive.driveCartesian(0,-0.3,0,0.0); // drive forwards half speed
    }else if (m_timer.get() > 9.0 && m_timer.get() < 10.0 ) {
      m_robotDrive.stopMotor();
    }else if (m_timer.get() > 11.0 && m_timer.get() < 12.0 ) {
      m_robotDrive.driveCartesian(-0.3,0,0,0.0); // drive forwards half speed
    }else {
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

    boolean forward = m_stick.getRawButton(fwd_button);
    boolean lateral = m_stick.getRawButton(ltr_button);
    boolean gyro_btn = m_stick.getRawButton(gyro_button);
    boolean inc = m_stick.getRawButton(inc_button);
    boolean dec = m_stick.getRawButton(dec_button);

    // Gyro
    double turningValue = (kAngleSetpoint - m_gyro.getAngle()) * kP; // Get angle
    turningValue = Math.copySign(turningValue, m_joystick.getY()); // Invert the direction of the turn if we are going backwards

    x_coef =  .6;
    y_coef = -.6;
    t_coef = .6;
    ag_coef = .01;

    triggers = (-1*r_stick.getTriggerAxis(Hand.kRight))+(r_stick.getTriggerAxis(Hand.kLeft));

    
    // forward button case
    if (forward) {
       m_robotDrive.driveCartesian(0, -1 * m_stick.getY(), 0, 0.0);
    } 
    
    // lateral button case
    else if (lateral){
       m_robotDrive.driveCartesian(m_stick.getX(), 0, 0, 0.0);
    }

    // increse agility
    else if (inc){
      if (x_coef == 1){System.out.println("x_coef is max");}
      else if (y_coef == -1){System.out.println("y_coef is max");}
      else if (t_coef == 1){System.out.println("t_coef is max");}
      else {
        x_coef += ag_coef;
        y_coef -= ag_coef;
        t_coef += ag_coef;
        System.out.println("x_coef = "+x_coef+", y_coef = "+y_coef+", t_coef = "+t_coef);
      }
    }

    // decrease agility
    else if (dec){
      if (x_coef == -1){System.out.println("x_coef is min");}
      else if (y_coef == 1){System.out.println("y_coef is min");}
      else if (t_coef == -1){System.out.println("t_coef is min");}
      else {
        x_coef -= ag_coef;
        y_coef += ag_coef;
        t_coef -= ag_coef;
        System.out.println("x_coef = "+x_coef+", y_coef = "+y_coef+", t_coef = "+t_coef);
      }
    }
     
    else {

      // eliminate z axis uncertainty
      if (Math.abs(m_stick.getZ()) > 0.2){
        System.out.println(m_stick.getZ());
        m_robotDrive.driveCartesian(x_coef*m_stick.getX(), y_coef*m_stick.getY(), t_coef*m_stick.getZ(), 0.0);
      }
      
      else {       

        // Gyro Code Starts Here
        if(gyro_btn){
          m_robotDrive.driveCartesian(x_coef*m_stick.getX(), y_coef*m_stick.getY(), t_coef*m_stick.getZ(), turningValue);
        }

        // standard drive
        else {
          m_robotDrive.driveCartesian(x_coef*m_stick.getX(), y_coef*m_stick.getY(), t_coef*m_stick.getZ(), 0.0);
        }
        
      }
      
    }
  
    r_elevator.set(triggers);
    r_roller.set(r_stick.getY(Hand.kLeft));
    r_lShooter.set(r_stick.getY(Hand.kRight));
    r_rShooter.set(r_stick.getY(Hand.kRight));
    r_intake.set((r_stick.getAButton()) ? 1 : 0 ); //Short Hand If...Else (Ternary Operator)
   
  
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
