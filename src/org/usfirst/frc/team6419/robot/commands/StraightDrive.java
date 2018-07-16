package org.usfirst.frc.team6419.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team6419.robot.Robot;

/**
 *
 */
public class StraightDrive extends Command {
	
	private boolean previousFieldRelative;

    public StraightDrive() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.drivetrain);
    	previousFieldRelative = Robot.drivetrain.fieldRelative;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.drivetrain.fieldRelative = false;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.drivetrain.drive(0.0, Robot.m_oi.getRightY(), 0.0);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return true;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.drivetrain.fieldRelative = previousFieldRelative;
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
