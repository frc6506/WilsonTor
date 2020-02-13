/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.controller.PIDController;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;

public class RotateToAngle extends Command {
  private double P = 0.04;
  private double I = 0.0;
  private double D = 0.0055;
  private PIDController pid = new PIDController(P, I, D);
  private double gryoSetPoint;
  private double commandStartTime;

  public RotateToAngle(double gsp) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.drivetrain);
    gryoSetPoint = gsp;
    pid.setTolerance(0.05 * gryoSetPoint);
    commandStartTime = Timer.getFPGATimestamp();
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.drivetrain.calibrate();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.drivetrain.drive(0, pid.calculate(Robot.drivetrain.gyro.getAngle(), gryoSetPoint));
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if (Timer.getFPGATimestamp() > commandStartTime + 2) {
      return true;
    } else {
      return pid.atSetpoint();
    }
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {}

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {}
}
