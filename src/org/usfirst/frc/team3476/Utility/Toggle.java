package org.usfirst.frc.team3476.Utility;

public class Toggle
{
	boolean rising, last, out;
	
	/**
	 * Constructs a Toggle that toggles on an edge based on rise (rising edge if rise == true, falling edge if rise == false).
	 * @param rise what edge to detect
	 */
	public Toggle(boolean rise)
	{
		rising = rise;
		last = false;
		out = false;
	}
	
	/**
	 * Constructs a Toggle that toggles on a rising edge.
	 */
	public Toggle()
	{
		rising = true;
		last = false;
		out = false;
	}
	
	/**
	 * Gets the current Toggle output state.
	 * @return the output state
	 */
	public boolean get()
	{
		return out;
	}
	
	/**
	 * Give the toggle a new input value.
	 * @param in the input value
	 */
	public void input(boolean in)
	{
		if(!rising)
		{
			inputFalling(in);
		}
		else
		{
			inputRising(in);
		}
		last = in;
	}
	
	/**
	 * The input helper method for rising edge mode.
	 * @param in the input value
	 */
	private void inputRising(boolean in)
	{
		if(in && !last)
		{
			out = !out;
		}
	}
	
	/**
	 * The input helper method for falling edge mode.
	 * @param in the input value
	 */
	private void inputFalling(boolean in)
	{
		if(!in && last)
		{
			out = !out;
		}
	}
}