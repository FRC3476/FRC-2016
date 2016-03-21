package org.usfirst.frc.team3476.Utility;

public class ManualHandler
{
	private long lastManualTime, timeThreshold;
	private boolean first;
	
	/**
	 * @param timeThresholdin the time threshold before considering this to be non manual
	 */
	public ManualHandler(long timeThresholdin)
	{
		timeThreshold = timeThresholdin;
		lastManualTime = 0;
	}
	
	/**
	 * Essentially !manual
	 * @return True if the time threshold has been passed
	 */
	public boolean isTimeUp()
	{
		return (System.currentTimeMillis() - lastManualTime) > timeThreshold;
	}
	
	/**
	 * Returns true only the first time after a poke
	 * @return
	 */
	public boolean first()
	{
		if(first)
		{
			first = false;
			return true;
		}
		else
		{
			return false;
		}
	}
	
	/**
	 * Manual method has run, so the time should be updated.
	 */
	public void poke()
	{
		first = true;
		lastManualTime = System.currentTimeMillis();
	}
	
	public void kill()
	{
		lastManualTime = System.currentTimeMillis() - timeThreshold - 1;
	}
}
