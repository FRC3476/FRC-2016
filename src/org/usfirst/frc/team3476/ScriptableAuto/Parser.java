package org.usfirst.frc.team3476.ScriptableAuto;

import java.io.IOException;
import java.util.ArrayList;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class Parser
{
	final String BACKUPCONSTANTS = "//Driving deadzones\nDRIVEDEAD = 2\nDRIVESTRAIGHTDEAD = 5\nTURNDEAD = 5\n\n//encoders\nUSELEFT = 1\nUSERIGHT = 1\n\n//Drivestraight PID\nSTRAIGHTP = 1.5\nSTRAIGHTI = 0\nSTRAIGHTD = 0.001\n\n//Drive PID\nDRIVEP = 1.3\nDRIVEI = 0.005\nDRIVED = 0\n\n//Turn PID\nTURNP = 1.7\nTURNI = 0.005\nTURND = 0\n\n//Intake constants\nSUCKMOTORSPEED = -1\nLOADMOTORSPEED = -1\nFORWARDISDOWN = 0 //false\nAIMUPPOWERED = 1 //true\n\n//Shooter constants\nSHOOTEROUTPUTRANGEHIGH = 1\nSHOOTEROUTPUTRANGELOW = -1\nSHOOTERIGAIN = 0.00001\nFLY1DIR = -1\nFLY2DIR = 1\nFLY3DIR = -1\nFLY4DIR = 1\nGRABFRISBEETIME = 0.65\nSHOOTFRISBEETIME = 0.33\nFLYWHEELDEAD = 100\nFLYWHEELMAXSPEED = 3000";
	
	final String PARALLELSEPARATOR = ";", CONSTANTSSEPERATOR = "\u001B", YEARSEPERATOR = "~", FIRSTPARAM = ":", SECONDPARAM = "@", THEN = ">";
	String script;
	String constants;
	String constantYear;
	
	public Parser(String scriptin, String constantsin, String thisYear)
	{
		script = scriptin;
		constantYear = thisYear;
		constants = retrieveThisYear(constantsin);
	}
	
	public ArrayList<CommandBlock> nextLine()
	{
		if(script.equals(""))
		{
			return new ArrayList<CommandBlock>();
		}
		int endOfLine = script.indexOf("\n");
		if(endOfLine == -1)
		{
			endOfLine = script.length();
		}
		String line = script.substring(0, endOfLine);//Get the next line
		if (script.length() == endOfLine)
		{
			script = "";//Remove the line we just retrieved
		}
		else
		{
			script = script.substring(endOfLine + 1);
		}
		
		//Remove comments
		int comdex = line.indexOf("//");
		if(comdex != -1)
		{
			line = line.substring(0, comdex);
		}
		
		ArrayList<CommandBlock> lineCommands = new ArrayList<CommandBlock>();
		
		int semiIndex = line.indexOf(PARALLELSEPARATOR);
		while(semiIndex != -1)//While there are still parallel commands to be processed, create command blocks
		{
			line = line.trim();
			String commandBlock = line.substring(0, semiIndex);
			line = line.substring(semiIndex + 1);
			
			lineCommands.add(parseCommandBlock(commandBlock));
			
			semiIndex = line.indexOf(PARALLELSEPARATOR);
		}
		lineCommands.add(parseCommandBlock(line));
		
		return lineCommands;
	}
	
	private CommandBlock parseCommandBlock(String commandBlock)
	{
		commandBlock = commandBlock.trim();
		ArrayList<Command> blockCommands = new ArrayList<Command>();
		
		int thendex = commandBlock.indexOf(THEN);
		while(thendex != -1)
		{
			commandBlock = commandBlock.trim();
			String command = commandBlock.substring(0, thendex);
			commandBlock = commandBlock.substring(thendex + 1);
			
			blockCommands.add(parseCommand(command));
			
			thendex = commandBlock.indexOf(THEN);
		}
		blockCommands.add(parseCommand(commandBlock));
		
		return new CommandBlock(blockCommands);
	}
	
	private Command parseCommand(String thenBlock)
	{
		thenBlock = thenBlock.trim();
		String command = "error";
		double colonParam = 0.0;
		double atParam = 0.0;
		int colonIndex = thenBlock.indexOf(FIRSTPARAM);
		int atIndex = thenBlock.indexOf(SECONDPARAM);
		if(colonIndex != -1)//Is there a colon?
		{
			command = thenBlock.substring(0, colonIndex).trim();
			if(atIndex != -1)//Is there an at?
			{
				colonParam = cleanDoubleParse(thenBlock.substring(colonIndex + 1, atIndex)); //Grab the stuff between the : and @ and parse
				atParam = cleanDoubleParse(thenBlock.substring(atIndex + 1)); //Grab the stuff after the @ and parse
			}
			else
			{
				colonParam = cleanDoubleParse(thenBlock.substring(colonIndex + 1)); //Grab the stuff after the : - no @
			}
		}
		else
		{
			command = thenBlock;
		}
		
		//Construct command from parsed command block
		double[] params = new double[2];
		params[0] = colonParam;
		params[1] = atParam;
		return new Command(command, params);
	}
	
	public double getConstant(String key) throws IOException
	{
		Matcher match = Pattern.compile("^" + key + "\\s*=", Pattern.MULTILINE).matcher(constants);
		
		int keydex = match.find() ? match.end() : -1;
		
		if(keydex == -1)
		{
			System.out.println("Match not found for key " + key);
			//throw new IOException("Missing key \"" + key + "\" in constants file: \"" + constantYear + "\"\nConstants in use: \"" + constants + "\"");
		}
//		if(equals == -1)
//		{
//			throw new IOException("Missing equals in " + constantYear + " constants at line " + (constants.substring(0, keydex).replace("[^\n]", "").length() + 1));
//		}
		int endOfLine = constants.indexOf("\n", keydex);
		if(endOfLine == -1)
		{
			endOfLine = constants.length();
		}
		String sValue = constants.substring(keydex, endOfLine).trim();
		return cleanDoubleParse(sValue);
	}
	
	private String retrieveThisYear(String constantsin)
	{
		ArrayList<String> years = new ArrayList<String>();
		int sep = constantsin.indexOf(CONSTANTSSEPERATOR);
		
		while(sep != -1)
		{
			years.add(constantsin.substring(0, sep));
			constantsin = constantsin.substring(sep + 1);
			
			sep = constantsin.indexOf(CONSTANTSSEPERATOR);
		}
		years.add(constantsin);
		
		for(String possYear : years)
		{
			if(possYear.substring(0, possYear.indexOf(YEARSEPERATOR)).trim().equals(constantYear))
			{
				return possYear.substring(possYear.indexOf(YEARSEPERATOR) + 1);
			}
		}
		System.out.println("USING BACKUP CONSTANTS!!!!!!!!!!!!!!");
		return BACKUPCONSTANTS;
	}
	
	private double cleanDoubleParse(String mess)
	{
		return Double.parseDouble(mess.replaceAll("[^\\d.-]", ""));
	}
	
	public boolean hasNextLine()
	{
		return !script.equals("");
	}
	
	public String getScript()
	{
		return script;
	}
	
	public String getConstants()
	{
		return constants;
	}
	
	public void update(String scriptin, String constantsin)
	{
		script = scriptin;
		constants = constantsin;
	}
}
