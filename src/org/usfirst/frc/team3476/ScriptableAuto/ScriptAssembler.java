package org.usfirst.frc.team3476.ScriptableAuto;


import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

public class ScriptAssembler
{
	public final String SUBCALL = "#";
	public final int MAXCALLS = 10;
	
	private String assembled, selected;
	private Map<String, String> scripts;
	private int calls;

	public ScriptAssembler()
	{
		this.assembled = "";
		this.selected = "";
		this.scripts = new HashMap<String, String>();
		this.calls = 0;
	}


	public void update(Map<String, String> scripts, String selected)
	{
		this.scripts = scripts;
		this.selected = selected;
		assemble();
	}
	
	private void assemble()
	{
		calls = 0;
		assembled = subassemble(selected);
	}
	
	private String subassemble(String scriptName)
	{
		if(calls++ > MAXCALLS)
		{
			throw new IllegalStateException("Too many recursive calls in \"" +
							selected + "\" at \"" + scriptName + "\" : " + calls);
		}
		
		String script = getScript(scriptName);
		
		String[] split = script.split(SUBCALL);
		String assembling = split[0];
		
		String[] pieces = Arrays.copyOfRange(split, 1, split.length);
		
		for(String piece : pieces)
		{
			int lineend = piece.indexOf("\n");
			
			if(lineend == -1)//end of file
			{
				assembling += subassemble(piece);
			}
			else//in file
			{
				String subScript = piece.substring(0, lineend);
				String rest = piece.substring(lineend);
				assembling += subassemble(subScript) + rest;
			}
		}
		
		return assembling;
	}
	
	private String getScript(String key)
	{
		if(!scripts.containsKey(key))
		{
			throw new IllegalScriptException("Raw script \"" + key + "\" not found in scripts");
		}
		//contains key, return value
		return scripts.get(key);
	}
	
	public String getAssembled()
	{
		return assembled;
	}
	
	public class IllegalScriptException extends RuntimeException
	{
		public IllegalScriptException(String string)
		{
			super(string);
		}
	}
}
