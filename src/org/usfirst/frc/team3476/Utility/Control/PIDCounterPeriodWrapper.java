package org.usfirst.frc.team3476.Utility.Control;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class PIDCounterPeriodWrapper extends Counter implements PIDSource
{
	double UNITSPERTICK;
	
	public PIDCounterPeriodWrapper(DigitalInput in, double distancepertickin)
	{
		super(in);
		UNITSPERTICK = distancepertickin;
	}
	
	public PIDCounterPeriodWrapper(int channel, double distancepertickin)
	{
		super(channel);
		UNITSPERTICK = distancepertickin;
	}
	
	@Override
	public double pidGet()
	{
		return UNITSPERTICK/getPeriod();
	}
	
	@Override
	public void setPIDSourceType(PIDSourceType pidSource){}

	@Override
	public PIDSourceType getPIDSourceType()
	{
		return PIDSourceType.kRate;
	}
}
