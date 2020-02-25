/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.revrobotics.ColorMatch;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Robot;

public class Position extends Command {
  public Position() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.sensor);
  }

  public Color initialColor;
  public Color testedColor;
  public Color prevColor;
  public int colorCount = 0;
  private final Color BLUE = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private final Color GREEN = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color RED = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private final Color YELLOW = ColorMatch.makeColor(0.361, 0.524, 0.113);

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    System.out.println("command Position initialized");
    Robot.sensor.turn(0.5);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    
    testedColor = Robot.sensor.getColor();
    if (testedColor.equals(initialColor) && !(testedColor.equals(prevColor))) {
      colorCount++;
    }
    prevColor = Robot.sensor.getColor();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    String gameData;
    gameData = DriverStation.getInstance().getGameSpecificMessage();
    if (gameData.length() > 0) {
      switch (gameData.charAt(0)) {
        case 'B':
          if (Robot.sensor.getColor().equals(RED)) {
            return true;
          }
          break;
        case 'G':
          if (Robot.sensor.getColor().equals(YELLOW)) {
            return true;
          }
          break;
        case 'R':
          if (Robot.sensor.getColor().equals(BLUE)) {
            return true;
          }
          break;
        case 'Y':
          if (Robot.sensor.getColor().equals(GREEN)) {
            return true;
          }
          break;
        default:
          return true;
      }
    } else {
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
