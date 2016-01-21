package org.usfirst.frc.team3476.Subsystems;

import org.usfirst.frc.team3476.Main.Subsystem;

public class StartSubsystem
{
	public static void init(Subsystem robotDriveSystem)
	{
		System.out.println("init");
		for(String command : robotDriveSystem.getAutoCommands())
		{
			if(command.toLowerCase().indexOf("clear") != -1)
			{
				robotDriveSystem.doAuto(null, command);
				return;
			}
		}
		throw new NullPointerException("No \"clear\" command in drive subsystem to appease watchdog.");
	}
}
