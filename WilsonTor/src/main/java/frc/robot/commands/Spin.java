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
import frc.robot.OI;

public class Spin extends Command {
  public Spin() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.sensor);
  }

  public Color initialColor;
  public Color currentColor;
  public Color previousColor;
  public int halfSpins = 0;
  public int fullSpins = 0;

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    initialColor = Robot.sensor.getColorMatch();
    previousColor = Robot.sensor.getColorMatch();
    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    //System.out.println("running Spin execute");
    Robot.sensor.commandHalfSpins = halfSpins;
    Robot.sensor.commandFullSpins = fullSpins;

    currentColor = Robot.sensor.getColorMatch();

    // if sensor sees initial color and the wheel has moved, then the wheel has turned halfway
    /*
    if (currentColor.equals(initialColor) && !(currentColor.equals(previousColor))) {
      halfSpins++;
      // update number of full spins
      fullSpins = (int) (halfSpins / 2);
    }
    */

    previousColor = Robot.sensor.getColorMatch();
    Robot.sensor.turn(0.5);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    // stop spinning when done enough spins
    if (fullSpins > 3) {Robot.sensor.turn(0);
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
