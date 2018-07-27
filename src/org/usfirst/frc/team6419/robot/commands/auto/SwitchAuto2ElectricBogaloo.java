package org.usfirst.frc.team6419.robot.commands.auto;

import org.usfirst.frc.team6419.robot.commands.OpenIntake;
import org.usfirst.frc.team6419.robot.commands.OuttakeCube;
import org.usfirst.frc.team6419.robot.commands.core.RaiseLiftForTime;
import org.usfirst.frc.team6419.robot.commands.core.TimedDriveStraight;
import org.usfirst.frc.team6419.robot.commands.core.TurnToHeading;
import org.usfirst.frc.team6419.robot.commands.core.Wait;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class SwitchAuto2ElectricBogaloo extends CommandGroup {

    public SwitchAuto2ElectricBogaloo() {
    	if (!Preferences.getInstance().containsKey("sw2-driveTime")) {
    		Preferences.getInstance().putDouble("sw2-driveTime", 2);
    	}
    	double driveTime = Preferences.getInstance().getDouble("sw2-driveTime", 2);
       
    	addSequential(new RaiseLiftForTime(5));
    	addSequential(new Wait(.5));
    	addSequential(new TimedDriveStraight(.25));
        if (DriverStation.getInstance().getGameSpecificMessage().charAt(0) == 'R') {
        	addSequential(new TurnToHeading(Math.toRadians(37)), 3);
        	//addParallel(new RaiseLiftForTime(1.5));
        	addSequential(new TimedDriveStraight(driveTime));
        } else {
        	addSequential(new TurnToHeading(Math.toRadians(-37)), 3);
        	//addParallel(new RaiseLiftForTime(1.5));
    		addSequential(new TimedDriveStraight(driveTime));
        }
        addSequential(new TurnToHeading(0), 3);
        addSequential(new TimedDriveStraight(1.5));
        addSequential(new OpenIntake());
        
    }
}
