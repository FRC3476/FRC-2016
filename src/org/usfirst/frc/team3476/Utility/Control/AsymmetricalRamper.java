package org.usfirst.frc.team3476.Utility.Control;


public class AsymmetricalRamper extends Ramper
{
	private double accelRate, decelRate;//units/second
	private boolean unitspersecond;
	private boolean dirty;

	public AsymmetricalRamper()
	{
		this(0, 0, true, 0);
	}
	
	public AsymmetricalRamper(double accelRate, double decelRate)
	{
		this(accelRate, decelRate, true, 0);
	}
	
	public AsymmetricalRamper(double accelRate, double decelRate, boolean unitspersecond)
	{
		this(accelRate, decelRate, unitspersecond, 0);
	}
	
	public void setAccelRate(double accelRate)
	{
		if(unitspersecond)
		{
			this.accelRate = accelRate;
		}
		else
		{
			this.accelRate = 1/accelRate;
		}
	}

	public void setDecelRate(double decelRate)
	{
		if(unitspersecond)
		{
			this.decelRate = decelRate;
		}
		else
		{
			this.decelRate = 1/decelRate;
		}
	}
	
	public void setUnitspersecond(boolean unitspersecond)
	{
		this.unitspersecond = unitspersecond;
	}

	public AsymmetricalRamper(double accelRate, double decelRate, boolean unitspersecond, double startval)
	{
		super(startval);
		this.unitspersecond = unitspersecond;
		if(unitspersecond)
		{
			this.accelRate = accelRate;
			this.decelRate = decelRate;
		}
		else
		{
			this.accelRate = 1/accelRate;
			this.decelRate = 1/decelRate;
		}
		dirty = false;
	}
	
	public void calc(double target)
	{
		//get the time diff in ns, to sec, time units per sec to get units
		double timeDiff;
		if(isFirst())
		{
			timeDiff = 1/1E9;
			setFirst(false);
		}
		else
		{
			timeDiff = (System.nanoTime() - getLastTime())/1E9;
		}
		setLastTime(System.nanoTime());
		double deltaTarget = target - getRampedval();
		int dir;
		if(deltaTarget != 0)
		{
			dir = deltaTarget < 0 ? -1 : 1;
		}
		else
		{
			dir = 0;
		}
		
		double delta;
		//if deltaTarget and rampedval different, decel, otherwise accel
		if(getRampedval()*dir < 0)//decel
		{
			delta = decelRate*timeDiff*dir;
		}
		else//accel
		{
			delta = accelRate*timeDiff*dir;
		}
		
		double checkDelta = delta - deltaTarget;
		
		if(checkDelta*delta > 0)//signs the same, target too close, set to target - end case
		{
			setRampedval(target);
		}
		else//move by the allowed amount
		{
			addToRampedval(delta);
		}
	}
}
