package org.usfirst.frc.team3476.Utility;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

public class ShooterLogger
{
	enum LoggingMode{FILE, STRING}
	private LoggingMode mode;
	
	private File logFile = new File("/usr/local/frc/logs/shooter data.txt");
	private FileWriter logger;
	
	private String building;
	
	private String BEFORE, AFTER, DELIM;
	
	private String prepped;
	
	private boolean first;
	
	public ShooterLogger()
	{
		this("{", "}", ", ", LoggingMode.STRING);
	}
	
	public ShooterLogger(String delim)
	{
		this("{", "}", delim, LoggingMode.STRING);
	}
	
	public ShooterLogger(String before, String after)
	{
		this(before, after, ", ", LoggingMode.STRING);
	}
	
	public ShooterLogger(String before, String after, String delim, LoggingMode modein)
	{
		mode = modein;
		switch(mode)
		{
			case FILE:
				try
				{
					if(logFile.createNewFile())
					{
						logger = new FileWriter(logFile);
					}
				} 
				catch (IOException e)
				{
					e.printStackTrace();
				}
				break;
				
			case STRING:
				building = "";
				break;
		}
		
		BEFORE = before;
		AFTER = after;
		DELIM = delim;
		
		prepped = "";
		
		first = true;
	}
	
	public void prepValue(double dist, double tach)
	{
		prepped = BEFORE + dist + ", " + tach + AFTER;//def is {dist, tach}
	}
	
	public void logPrepped()
	{
		if(valueToLog())
		{
			String toLog = (first ? "" : DELIM) + prepped;
			switch(mode)
			{
				case FILE:
					try
					{
						logger.write(toLog);
					} 
					catch (IOException e)
					{
						e.printStackTrace();
					}
					break;
					
				case STRING:
					building += toLog;
					break;
					
			}
			clearPrepped();
			first = false;
		}
		else
		{
			System.out.println("Nothing to log");
		}
	}
	
	public boolean valueToLog()
	{
		return prepped.length() != 0;
	}
	
	public void clearPrepped()
	{
		prepped = "";
	}
	
	public void clearLogger()
	{
		switch(mode)
		{
			case FILE:
				System.out.println("Invalid logger operation");
				
			case STRING:
				building = "";
		}
	}
	
	public String toString()
	{
		switch(mode)
		{
			case FILE:
				return "Invalid logger operation";
				
			case STRING:
				return building;
				
			default:
				return "mode is not FILE or STRING";
		}
	}
}
