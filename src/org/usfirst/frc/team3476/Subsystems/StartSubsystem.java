package org.usfirst.frc.team3476.Subsystems;

import org.usfirst.frc.team3476.Main.Subsystem;

public class StartSubsystem
{
	public static void init(Subsystem robotDriveSystem)
	{
		//System.out.println("init");
		for(String command : robotDriveSystem.getAutoCommands())
		{
			if(command.toLowerCase().indexOf("clear") != -1)
			{ 
				//System.out.println("command");
				robotDriveSystem.doAuto(null, command);
				//System.out.println("Drive System");
				return;
			}
		}
		throw new NullPointerException("No \"clear\" command in drive subsystem to appease watchdog.");
	}
}
