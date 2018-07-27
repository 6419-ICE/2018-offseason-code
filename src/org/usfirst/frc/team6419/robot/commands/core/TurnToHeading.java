package org.usfirst.frc.team6419.robot.commands.core;

import org.usfirst.frc.team6419.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Sets the drivetrain's target heading
 */
public class TurnToHeading extends Command {
	
	private double _heading;
	
	/**
	 * Construct a TurnToHeading command
	 * @param heading target heading
	 */
    public TurnToHeading(double heading) {
    	requires(Robot.drivetrain);
    	_heading = heading;
    }

    protected void initialize() {
    	Robot.drivetrain.setTargetHeading(_heading);
    }

    protected void execute() {
    	Robot.drivetrain.drive(0, 0, 0);
    }

    protected boolean isFinished() {
        return Robot.drivetrain.targetHeadingReached();
    }

    protected void end() {
    	Robot.drivetrain.stop();
    }

    protected void interrupted() {
    	end();
    }
}
