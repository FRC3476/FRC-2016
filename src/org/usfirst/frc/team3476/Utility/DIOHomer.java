package org.usfirst.frc.team3476.Utility;

import edu.wpi.first.wpilibj.DigitalInput;

public class DIOHomer
{
	private double SEEKSPEED, SLOWSPEED, HOMINGDIR, curspeed;
	private long TIMEOUT, starttime;
	private boolean HOMEHIGH, opposite, lasttriggered;
	private DigitalInput dio;
	public enum HomeState{SEEK, HOME, TIMEOUT}
	HomeState state;
	
	public DIOHomer(DigitalInput dioin, double SEEKSPEEDIN,
			double SLOWSPEEDIN, double HOMINGDIRIN, boolean HOMEHIGHIN, long TIMEOUTIN)
	{
		state = HomeState.SEEK;
		dio = dioin;
		SEEKSPEED = SEEKSPEEDIN;
		SLOWSPEED = SLOWSPEEDIN;
		HOMINGDIR = HOMINGDIRIN;
		HOMEHIGH = HOMEHIGHIN;
		TIMEOUT = TIMEOUTIN;
		opposite = false;
		lasttriggered = false;
		starttime = System.currentTimeMillis();
	}
	
	public HomeState getState()
	{
		return state;
	}
	
	public boolean isHome()
	{
		return state == HomeState.HOME;
	}
	
	public boolean isStopped()
	{
		return state == HomeState.HOME || state == HomeState.TIMEOUT;
	}
	
	public void update()
	{
		if(!opposite && System.currentTimeMillis() - starttime > TIMEOUT)
		{
			starttime = System.currentTimeMillis();
			opposite = true;
		}
		else if(System.currentTimeMillis() - starttime > TIMEOUT*2)
		{
			state = HomeState.TIMEOUT;
		}
		
		boolean triggered = !(dio.get()^HOMEHIGH);
		
		switch(state)
		{
			case SEEK:
				curspeed = SEEKSPEED*HOMINGDIR*(opposite ? -1:1);
				
				if(opposite)
				{
					if(!triggered && lasttriggered)//falling edge
					{
						state = HomeState.HOME;
					}
					else
						break;
				}
				else
				{
					if(triggered && !lasttriggered)//rising edge
					{
						state = HomeState.HOME;
					}
					else
						break;
				}
				
			case HOME:
				curspeed = 0;
				break;
				
			default:
				curspeed = 0;
				break;
		}
		lasttriggered = triggered;
	}
	
	public double getSpeed()
	{
		return curspeed;
	}
	
	public void restart()
	{
		state = HomeState.SEEK;
		starttime = System.currentTimeMillis();
		opposite = false;
		System.out.println("Reset");
	}
	
	public void kill()
	{
		state = HomeState.HOME;
	}
	
	@Override
	public boolean equals(Object in)
	{
		if(in == null)return false;
		boolean result = true;
		DIOHomer other = (DIOHomer)in;
		result = result && OrangeUtility.doubleEqual(this.SEEKSPEED, other.SEEKSPEED);
		result = result && OrangeUtility.doubleEqual(this.SLOWSPEED, other.SLOWSPEED);
		result = result && OrangeUtility.doubleEqual(this.HOMINGDIR, other.HOMINGDIR);
		result = result && OrangeUtility.doubleEqual(this.TIMEOUT, other.TIMEOUT);
		result = result && this.HOMEHIGH == other.HOMEHIGH;
		return result;
	}
}
