package org.usfirst.frc.team3476.Utility;

import java.util.Arrays;
import java.util.List;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import org.usfirst.frc.team3476.Utility.Control.ControlLoop;

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
	
	public static boolean inRange(double test, double a, double b)
	{
		return inRange(test, new double[]{a, b});
	}

	public static boolean inRange(double test, double[]range)
	{
		return test > getArrayMin(range) && test < getArrayMax(range);
	}
	
	public static double getArrayMin(double[] array)
	{
		double result = array[0];
		for(double e : array)
		{
			if(e < result)
			{
				result = e;
			}
		}
		return result;
	}
	
	public static double getArrayMax(double[] array)
	{
		double result = array[0];
		for(double e : array)
		{
			if(e > result)
			{
				result = e;
			}
		}
		return result;
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
		if(toDonut == 0)
		{
			return 0;
		}
		if(toDonut > 0)
		{
			return toDonut + threshold;
		}
		return toDonut - threshold;
	}
	
	public static double scalingDonut(double toDonut, double threshold, double clamp, double maxInput)
	{
		threshold = Math.abs(threshold);
		clamp = Math.abs(clamp);
		maxInput = Math.abs(maxInput);
		if(toDonut == 0)
		{
			return 0;
		}
		
		//range check
		if(toDonut > clamp || toDonut > maxInput)
		{
			return Math.min(clamp, maxInput);
		}
		if(toDonut < -clamp || toDonut < -maxInput)
		{
			return -Math.min(clamp, maxInput);
		}
		
		//calc factors
		double newRange = maxInput - threshold;
		double factor = newRange/maxInput;//maxInput is the old range (maxInput - 0)

		if(toDonut > 0)
		{
			return toDonut*factor + threshold;
		}
		return toDonut*factor - threshold;
	}
	
	/**
	 * Extracts the double value from a string.
	 * @param mess the string to be parsed
	 * @return the double value extracted from the string
	 */
	public static double cleanDoubleParse(String mess)//TODO: encapsulate special replaceall
	{
		String result = mess;
		result = result.toUpperCase();
		result = result.replaceAll("[^\\d.\\-E]", "");//remove all illegal characters
		int e = result.indexOf("E");
		if(e != -1)
		{
			String before = result.substring(0, e + 1), after = result.substring(e + 1);
			before = removeExtraInstances(before, "\\.");
			before = removeExtraInstances(before, "-");
			after = removeExtraInstances(after, "\\.");
			after = removeExtraInstances(after, "-");
			
			result = before + after;
		}
		else//no e
		{
			result = removeExtraInstances(result, "\\.");
			result = removeExtraInstances(result, "-");
		}
		
		return Double.parseDouble(result);
	}
	
	/**
	 * Removes all instances of toReplace after the first.
	 * If toReplace does not occur, input is returned unchanged.
	 * @param input the String to operate on
	 * @param replaceRegex the regex to remove extra instances of
	 * @return the resulting string
	 */
	public static String removeExtraInstances(String input, String replaceRegex)
	{
		Matcher match = Pattern.compile(replaceRegex).matcher(input);
		if(match.find())
		{
			return replaceAllPastIndex(input, replaceRegex, "", match.start());
		}
		else
		{
			return input;
		}
	}
	
	/**
	 * Calls input.replaceAll() on everything after the index
	 * Precondition: 0 <= index < input.length
	 * @param input the input String
	 * @param regex the regex to be found
	 * @param replacement the String to replace matching substrings
	 * @param index the index to start at (non-inclusive)
	 * @return the resulting string
	 */
	public static String replaceAllPastIndex(String input, String regex, String replacement, int index)
	{
		return input.substring(0, index + 1) + input.substring(index + 1).replaceAll(regex, replacement);
	}
	
	/**
	 * Removes single line comments
	 * @param input the String to operate on
	 * @param commentDelimiter the String that denotes a comment
	 * @return the modified String
	 */
	public static String removeSLComments(String input, String commentDelimiter)
	{
		int comdex = input.indexOf(commentDelimiter);
		int comend = input.indexOf("\n", comdex);
		while(comdex != -1)
		{
			if(comend != -1)//comend
			{
				input = input.substring(0, comdex) + input.substring(comend);
			}
			else//comment abbuts end of string
			{
				input = input.substring(0, comdex);
			}
			comdex = input.indexOf(commentDelimiter);
			comend = input.indexOf("\n", comdex);
		}
		return input;
	}
	
	/**
	 * Returns a string with all the parameters of the passed PID.
	 * @param control the PIDController to get info from.
	 * @return the info of the PIDController.
	 */
	public static String PIDData(PIDController control)
	{
		return PIDData(control, 0, 0);
	}
	
	/**
	 * Returns a string with all the parameters of the passed PID.
	 * @param control the PIDController to get info from.
	 * @return the info of the PIDController.
	 */
	public static String PIDData(PIDController control, double outlow, double outhigh)
	{
		boolean enable = control.isEnabled(), tol = control.onTarget();
		double error = control.getError(), process = control.getSetpoint() - control.getError(),
				conval = control.get(), p = control.getP(), i = control.getI(), d = control.getD();
		String ret = "Enable: " + enable;
		ret += ",\nError: " + error;
		ret += ",\nProcess: " + process;
		ret += ",\nControl Value: " + conval;
		ret += ",\nP: " + p;
		ret += ",\nI: " + i;
		ret += ",\nD: " + d;
		if(outlow != 0 || outhigh != 0) ret += ",\nOutput Range: " + Arrays.toString(new double[]{outlow, outhigh});
		ret += ",\nIn Tolerance: " + tol;
		ret += "\n==============";
		return ret;
	}
	
	/**
	 * Returns a string with all the parameters of the passed PID.
	 * @param control the PIDController to get info from.
	 * @return the info of the PIDController.
	 */
	public static String ControlLoopData(ControlLoop control, double process)
	{
		double error = control.getError(process), conval = control.output(process);
		String ret = "==============";
		ret += "\nError: " + error;
		ret += ",\nProcess: " + process;
		ret += ",\nControl Value: " + conval;
		ret += "\n==============";
		return ret;
	}
	
	public static String ControlData(double error, double process, double output)
	{
		String ret = "==============";
		ret += "\nError: " + error;
		ret += ",\nProcess: " + process;
		ret += ",\nControl Value: " + output;
		ret += "\n==============";
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
	
	public static boolean doubleEqual(double a, double b)
	{
		double epsilon = 1E-14;
		return Math.abs(a-b) < epsilon;
	}
}
