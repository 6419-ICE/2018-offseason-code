package org.usfirst.frc.team6419.robot.commands;

import org.usfirst.frc.team6419.robot.OI;
import org.usfirst.frc.team6419.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class HandleLift extends Command {

    public HandleLift() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.lift);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.lift.stop();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	int pov = Robot.m_oi.getLeftJoystick().getPOV(0);
    	if (pov != -1) {
    		if (pov > 270 || pov < 90) {
    			// up
    			Robot.lift.setPower(1);
    		} else if (pov > 90 && pov < 270) {
    			// down
    			Robot.lift.setPower(-0.5);
    		}
    	} else {
    		pov = Robot.m_oi.getRightJoystick().getPOV(0);
    		if (pov != -1) {
        		if (pov > 270 || pov < 90) {
        			// up
        			Robot.lift.setPower(1);
        		} else if (pov > 90 && pov < 270) {
        			// down
        			Robot.lift.setPower(-0.5);
        		}
    		} else {
    			Robot.lift.park();
    		}
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.lift.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
