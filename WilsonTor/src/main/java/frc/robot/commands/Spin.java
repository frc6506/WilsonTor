/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import edu.wpi.first.wpilibj.util.Color;

public class Spin extends Command {
  public Spin() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.sensor);
  }
  public Color initialColor;
  public Color testedColor;
  public Color prevColor;
  public int colorCount = 0;


  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    initialColor = Robot.sensor.getColor();
    Robot.sensor.turn(0.5);

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    testedColor = Robot.sensor.getColor();
    if(testedColor.equals(initialColor) && !(testedColor.equals(prevColor))) {
        colorCount++;
    }
    prevColor = Robot.sensor.getColor();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(colorCount>6) {
        return true;
    }
    return false;

  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.sensor.turn(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {}
}
