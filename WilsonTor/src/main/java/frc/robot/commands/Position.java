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
  public boolean passedInitialColor = false;
  private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    System.out.println("command Position initialized");
    initialColor = Robot.sensor.getColorMatch();
    passedInitialColor = false;
    Robot.sensor.turn(0.19);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {}

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    String gameData;
    gameData = "B";
    // gameData = DriverStation.getInstance().getGameSpecificMessage();
    if (!Robot.sensor.getColorMatch().equals(initialColor)) {
      passedInitialColor = true;
    }
    if (gameData.length() > 0) {
      switch (gameData.charAt(0)) {
        case 'B':
          // red
          // to hit red, stop at green
          if (Robot.sensor.getColorMatch().equals(kBlueTarget) && passedInitialColor) {
            Robot.sensor.turn(0); // change back to green
            return true;
          }
          break;
        case 'G':
          // yellow
          // to hit yellow, stop at red
          if (Robot.sensor.getColorMatch().equals(kRedTarget) && passedInitialColor) {
            Robot.sensor.turn(0);
            return true;
          }
          break;
        case 'R':
          // blue
          // to hit blue, stop at yellow
          if (Robot.sensor.getColorMatch().equals(kYellowTarget) && passedInitialColor) {
            Robot.sensor.turn(0); // change back to yellow
            return true;
          }
          break;
        case 'Y':
          // red
          // to hit red, stop at blue
          if (Robot.sensor.getColorMatch().equals(kBlueTarget) && passedInitialColor) {
            Robot.sensor.turn(0);
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
    // accounts for slight overshoot
    for (int i = 0; i < 2000; i++) {
      Robot.sensor.turn(-0.2);
    }

    Robot.sensor.turn(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {}
}
