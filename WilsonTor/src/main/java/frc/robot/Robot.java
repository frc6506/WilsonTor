/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.I2C; 
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.subsystems.Mailbox;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climb;
import frc.robot.utils.Limelight;
import edu.wpi.first.wpilibj.util.Color; 

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult; 
import com.revrobotics.ColorMatch; 

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static OI m_oi;

  // subsystems
  public static Mailbox mail = new Mailbox();
  public static Drivetrain drivetrain = new Drivetrain();
  public static Arm armMotor = new Arm();
  public static Climb climbDevice = new Climb();

  //color sensor
  private final I2C.Port i2cPort = I2C.Port.kOnboard; 
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch m_colorMatcher = new ColorMatch(); 
  
  private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240); 
  private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114); 
  private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113); 
 
  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_oi = new OI();
    drivetrain.initializeGyro();
    drivetrain.calibrate();

    m_colorMatcher.addColorMatch(kBlueTarget); 
    m_colorMatcher.addColorMatch(kGreenTarget); 
    m_colorMatcher.addColorMatch(kRedTarget); 
    m_colorMatcher.addColorMatch(kYellowTarget); 
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("tx", Limelight.returnHorizontalOffset());
    SmartDashboard.putNumber("ty", Limelight.returnVerticalOffset());

    Color detectedColor = m_colorSensor.getColor(); 

    String colorString; 
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor); 

    //color algorithm
    if(match.color == kBlueTarget){
      colorString = "Blue"; 
    }
    else if(match.color == kGreenTarget){
      colorString = "Green";
    }
    else if(match.color == kRedTarget){
      colorString = "Red"; 
    }
    else if(match.color == kYellowTarget){
      colorString = "Yellow";
    }
    else{
      colorString = "Unknown"; 
    }

    //putting the values onto Shuffleboard
    SmartDashboard.putNumber("Red", detectedColor.red); 
    SmartDashboard.putNumber("Green", detectedColor.green); 
    SmartDashboard.putNumber("Confidence", match.confidence); 
    SmartDashboard.putNumber("Blue", detectedColor.blue); 
    SmartDashboard.putString("DetectedColor", colorString); 


  }

  /**
   * This function is called once each time the robot enters Disabled mode. You can use it to reset
   * any subsystem information you want to clear when the robot is disabled.
   */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the chooser code above
   * (like the commented example) or additional comparisons to the switch structure below with
   * additional strings & commands.
   */
  @Override
  public void autonomousInit() {}

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
