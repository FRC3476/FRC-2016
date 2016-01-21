package org.usfirst.frc.team3476.Subsystems;

import org.usfirst.frc.team3476.Main.Subsystem;

public class SubsystemTask implements Runnable
{
	private Subsystem system;
	private boolean running, action;
	
	public SubsystemTask(Subsystem systemin)
	{
		system = systemin;
		running = true;
		action = false;
	}
	
	public synchronized void terminate()
	{
		running = false;
	}
	
	public synchronized void hold()
	{
		action = false;
	}
	
	public synchronized void resume()
	{
		action = true;
	}
	
	@Override
	public void run()
	{
		while(running)
		{
			if(action) system.update();
			else
				try
				{
					Thread.sleep(50);
				}
				catch (InterruptedException e)
				{
					e.printStackTrace();
				}
		}
	}

}
