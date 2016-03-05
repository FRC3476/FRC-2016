package org.usfirst.frc.team3476.Main;

import org.usfirst.frc.team3476.Main.SafeTask;
import org.usfirst.frc.team3476.Utility.OrangeUtility;

public abstract class SafeTask implements Runnable
{
	private boolean running, active;
	private int executionTime = 10;
	private long lastTime;
	private boolean printNext, printOnlyOnOverrun;
	
	private String format;
	private Object printParam;
	
	public SafeTask(int minTime)
	{
		running = true;
		active = false;
		printNext = false;
		printOnlyOnOverrun = false;
		executionTime = minTime;
		lastTime = 0;
		format = "[%s] Time: %d%n";
		printParam = "";
	}
	
	public void setPrePrint(String formatMessage, Object element)
	{
		format = "[" + formatMessage + "] Time: %d%n";
		printParam = element;
	}
	
	public void printNext(boolean onOverrun)
	{
		printNext = true;
		printOnlyOnOverrun = onOverrun;
	}
	
	public long getLastTime()
	{
		return lastTime;
	}
	
	public int getMinTime()
	{
		return executionTime;
	}
	
	public boolean ranOver()
	{
		return lastTime > executionTime;
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
				lastTime = time;
				if(printNext)
				{
					if(printOnlyOnOverrun)
					{
						if(ranOver())
						{
							System.out.printf(format, printParam, time);
						}
					}
					else
					{
						System.out.printf(format, printParam, time);
					}
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
