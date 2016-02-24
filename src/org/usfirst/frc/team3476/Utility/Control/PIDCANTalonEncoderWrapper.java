package org.usfirst.frc.team3476.Utility.Control;

import org.omg.IOP.ENCODING_CDR_ENCAPS;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class PIDCANTalonEncoderWrapper implements PIDSource, CounterBase
{
	CANTalon talon;
	double UNITSPERTICK;
	
	public PIDCANTalonEncoderWrapper(CANTalon talonin, double unitspertickin)
	{
		talon = talonin;
		UNITSPERTICK = unitspertickin;
	}
	
	@Override
	public double pidGet()
	{
		return talon.getPosition()*UNITSPERTICK;
	}
	

	@Override
	public void setPIDSourceType(PIDSourceType pidSource){}

	@Override
	public PIDSourceType getPIDSourceType(){return PIDSourceType.kDisplacement;}

	public void reset()
	{
		setPosition(0);
	}
	
	public void setPosition(double units)
	{
		talon.setPosition(units/UNITSPERTICK);
	}

	/**
	 * Returns actual ticks.
	 */
	@Override
	public int get()
	{
		return (int)talon.getPosition();
	}

	/**
	 * Do not use - returns 0.
	 */
	@Override
	public double getPeriod(){return 0;}

	/**
	 * Do not use.
	 */
	@Override
	public void setMaxPeriod(double maxPeriod){}

	/**
	 * Do not use - returns false.
	 */
	@Override
	public boolean getStopped() {
		return false;
	}

	@Override
	public boolean getDirection()
	{
		return talon.getSpeed() >= 0;
	}

	public double getDistance()
	{
		return pidGet();
	}
}
