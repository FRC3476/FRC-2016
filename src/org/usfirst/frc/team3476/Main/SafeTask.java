package org.usfirst.frc.team3476.Main;

import org.usfirst.frc.team3476.Main.SafeTask;
import org.usfirst.frc.team3476.Utility.OrangeUtility;

public abstract class SafeTask implements Runnable
{
	private boolean running, active;
	private int executionTime = 10;
	
	public SafeTask(int minTime)
	{
		running = true;
		active = false;
		executionTime = minTime;
	}
	
	/**
	 * Safely exits thread execution.
	 */
	public synchronized void terminate()
	{
		running = false;
	}
	
	/**
	 * Pauses thread execution.
	 */
	public synchronized void hold()
	{ 
		active = false;
	}
	
	/**
	 * Resumes thread execution.
	 */
	public synchronized void resume()
	{
		active = true;
	}
	
	/**
	 * @return True if this thread is active.
	 */
	public synchronized boolean isActive()
	{
		return active;
	}
	
	@Override
	/**
	 * Starts execution that can be paused and terminated at any time.
	 */
	public void run()
	{
		long time;
		while(running)
		{
			if(active)
			{
				time = System.currentTimeMillis();
				action();
				time = System.currentTimeMillis() - time;
				if(time < executionTime)
				{
					OrangeUtility.sleep(executionTime - time);
				}
			}
			else
				OrangeUtility.sleep(50);
		}
	}
	
	/**
	 * Performs a pauseable action.
	 */
	protected abstract void action();
}
