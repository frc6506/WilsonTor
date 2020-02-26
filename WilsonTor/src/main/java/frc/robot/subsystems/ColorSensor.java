/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

import frc.robot.RobotMap;
import frc.robot.commands.Spin;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import edu.wpi.first.wpilibj.util.Color;

/** Add your docs here. */
public class ColorSensor extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  VictorSPX sensorMotor = new VictorSPX(RobotMap.MOTOR_SENSOR_ID);

  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch m_colorMatcher = new ColorMatch();

  private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

  // variables from Spin
  public int commandHalfSpins = 0;
  public int commandFullSpins = 0;

  public ColorSensor() {
    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);
  }

  Color detectedColor = m_colorSensor.getColor();
  String colorString;
  ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

  public void updateColor() {
    detectedColor = m_colorSensor.getColor();
    match = m_colorMatcher.matchClosestColor(detectedColor);
  }

  public void reportColorToDashboard() {
    // color algorithm
    if (match.color == kBlueTarget) {
      colorString = "Blue";
    } else if (match.color == kGreenTarget) {
      colorString = "Green";
    } else if (match.color == kRedTarget) {
      colorString = "Red";
    } else if (match.color == kYellowTarget) {
      colorString = "Yellow";
    } else {
      colorString = "Unknown";
    }
    // putting the values onto Shuffleboard
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Confidence", match.confidence);

    SmartDashboard.putString("DetectedColor", colorString);
  }

  public void reportSpinsToDashboard() {
    SmartDashboard.putNumber("Half Spins", commandHalfSpins);
    SmartDashboard.putNumber("Full Spins", commandFullSpins);
    System.out.println("Half Spins: " + commandHalfSpins);
    System.out.println("Full Spins: " + commandHalfSpins);
  }

  // Wrapper class
  public void turn(double voltagePercent) {
    sensorMotor.set(ControlMode.PercentOutput, voltagePercent);
  }

  @Override
  public void initDefaultCommand() {
    // setDefaultCommand(new Spin());
  }

  public Color getColor() {
    return m_colorSensor.getColor();
  }

  public Color getColorMatch() {
    return match.color;
  }
}
