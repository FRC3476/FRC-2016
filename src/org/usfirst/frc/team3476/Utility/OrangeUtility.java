package org.usfirst.frc.team3476.Utility;

import java.util.List;

import edu.wpi.first.wpilibj.PIDController;

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
	
	/**
	 * Returns a string with all the parameters of the passed PID.
	 * @param control the PIDController to get info from.
	 * @return the info of the PIDController.
	 */
	public static String PIDData(PIDController control)
	{
		String ret = "Enable: " + control.isEnabled();
		ret += ",\nError: " + control.getError();
		ret += ",\nCurrent: " + (control.getSetpoint() + control.getError());
		ret += ",\nControl Value: " + control.get();
		ret += "\n";
		return ret;
	}
	
	/**
	 * Encapsulates Thread.sleep to make code more readable.
	 * @param millis the time to sleep
	 */
	public static void sleep(long millis)
	{
		try
		{
			Thread.sleep(millis);
		}
		catch (InterruptedException e)
		{
			e.printStackTrace();
		}
	}
	
	public static String joinStrings(String delim, List<?> strings)
	{
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < strings.size(); ++i)
        {
            sb.append(strings.get(i).toString());
            if (i < strings.size() - 1)
            {
                sb.append(delim);
            }
        }
        return sb.toString();
    }
}
