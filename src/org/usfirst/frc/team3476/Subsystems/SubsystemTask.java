package org.usfirst.frc.team3476.Subsystems;

import org.usfirst.frc.team3476.Main.SafeTask;
import org.usfirst.frc.team3476.Main.Subsystem;

public class SubsystemTask extends SafeTask
{
	private Subsystem system;
	
	public SubsystemTask(Subsystem systemin, int minTime)
	{
		super(minTime);
		system = systemin;
		setPrePrint("System {%s}", system);
	}

	@Override
	public synchronized void action()
	{
		system.update();
	}
}
