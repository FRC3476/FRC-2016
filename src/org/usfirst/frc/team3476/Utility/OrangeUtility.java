package org.usfirst.frc.team3476.Utility;

public class OrangeUtility
{
	/**
	 * Keeps a value in a range by truncating it.
	 * @param toCoerce the value to coerce
	 * @param high the high value of the range
	 * @param low the low value of the range
	 * @return the coerced value
	 */
	public static double coerce(double toCoerce, double high, double low)
	{
		if(toCoerce > high)
		{
			return high;
		}
		else if(toCoerce < low)
		{
			return low;
		}
		return toCoerce;
	}
	
	public static double normalize(double toNormalize, double fromHigh, double fromLow, double toHigh, double toLow)
	{
		double factor = (toHigh - toLow) / (fromHigh - fromLow);
		double add = toLow - fromLow*factor;
		return toNormalize*factor + add;
	}
	
	/**
	 * Checks  if it is legal to start a thread.
	 * @param testing the Thread to test
	 * @return if the Thread can be started
	 */
	public static boolean isStartLegal(Thread testing)
	{
		return testing.getState() == Thread.State.NEW;
	}
	
	/**
	 * Donuts an input value so that a control loop can overcome backlash or friction.
	 * @param toDonut the input value
	 * @param threshold the backlash or friction scalar
	 * @return the adjusted "input" value for evaluation by the control loop
	 */
	public static double donut(double toDonut, double threshold)
	{
		if(toDonut > 0)
		{
			return toDonut + threshold;
		}
		return toDonut - threshold;
	}
	
	/**
	 * Extracts the double value from a string.
	 * @param mess the string to be parsed
	 * @return the double value extracted from the string
	 */
	public static double cleanDoubleParse(String mess)
	{
		return Double.parseDouble(mess.replaceAll("[^\\d.-]", ""));
	}
}
