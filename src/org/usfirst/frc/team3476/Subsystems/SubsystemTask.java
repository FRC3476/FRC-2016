package org.usfirst.frc.team3476.Subsystems;

import org.usfirst.frc.team3476.Main.SafeTask;
import org.usfirst.frc.team3476.Main.Subsystem;

public class SubsystemTask extends SafeTask
{
	private Subsystem system;
	
	public SubsystemTask(Subsystem systemin)
	{
		system = systemin;
	}

	@Override
	public void action()
	{
		system.update();	
	}
}
