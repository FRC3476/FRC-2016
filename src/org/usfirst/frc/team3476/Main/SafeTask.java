package org.usfirst.frc.team3476.Main;

import org.usfirst.frc.team3476.Main.SafeTask;

public abstract class SafeTask implements Runnable
{
	private boolean running, action;
	
	public SafeTask()
	{
		running = true;
		action = false;
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
		action = false;
	}
	
	/**
	 * Resumes thread execution.
	 */
	public synchronized void resume()
	{
		action = true;
	}
	
	/**
	 * @return True if this thread is active.
	 */
	public synchronized boolean isActive()
	{
		return action;
	}
	
	@Override
	/**
	 * Starts execution that can be paused and terminated at any time.
	 */
	public void run()
	{
		while(running)
		{
			if(action) action();
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
	
	/**
	 * Performs a pauseable action.
	 */
	protected abstract void action();
}
