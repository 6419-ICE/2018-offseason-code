package org.usfirst.frc.team6419.robot.commands.auto;

import org.usfirst.frc.team6419.robot.Robot.Auto;
import org.usfirst.frc.team6419.robot.commands.OuttakeCube;
import org.usfirst.frc.team6419.robot.commands.core.DriveToPoint;
import org.usfirst.frc.team6419.robot.commands.core.RaiseLiftForTime;
import org.usfirst.frc.team6419.robot.commands.core.TimedDriveStraight;
import org.usfirst.frc.team6419.robot.commands.core.TurnToHeading;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class SwitchAutoPosition3 extends CommandGroup {

    public SwitchAutoPosition3(Auto pos) {
    	addSequential(new TimedDriveStraight(4));
    	if(DriverStation.getInstance().getGameSpecificMessage().charAt(0) == pos.toChar()) {
    		if (pos == Auto.LEFT) {
    			addSequential(new TurnToHeading(Math.toRadians(90)), 3);
    		}
    		else {
    			addSequential(new TurnToHeading(Math.toRadians(-90)), 3);
    		}
    		addSequential(new RaiseLiftForTime(1.555));
    		addSequential(new TimedDriveStraight(1));
    		addSequential(new OuttakeCube(), 2.5);
    	}
    }
}
