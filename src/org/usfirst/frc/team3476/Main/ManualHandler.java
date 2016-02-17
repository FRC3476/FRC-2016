package org.usfirst.frc.team3476.Main;

public class ManualHandler
{
	private long lastManualTime, timeThreshold;
	private boolean nopoke;
	
	/**
	 * @param timeThresholdin the time threshold before considering this to be non manual
	 */
	public ManualHandler(long timeThresholdin)
	{
		timeThreshold = timeThresholdin;
		nopoke = true;
	}
	
	/**
	 * Essentially !manual
	 * @return True if the time threshold has been passed
	 */
	public boolean isTimeUp()
	{
		if(nopoke)
		{
			return true;
		}
		return (System.currentTimeMillis() - lastManualTime) > timeThreshold;
	}
	
	/**
	 * Manual method has run, so the time should be updated.
	 */
	public void poke()
	{
		nopoke = false;
		lastManualTime = System.currentTimeMillis();
	}
}
