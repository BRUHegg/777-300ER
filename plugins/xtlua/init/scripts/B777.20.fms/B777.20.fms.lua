--[[
*****************************************************************************************
* Script Name: misc
* Author Name: Crazytimtimtim
* Script Description: (Init) misc system code for systems that don't have enough for their own module.
*****************************************************************************************
--]]

--replace create_command
function deferred_command(name,desc,nilFunc)
	c = XLuaCreateCommand(name,desc)
	--print("Deferred command: "..name)
	--XLuaReplaceCommand(c,null_command)
	return nil --make_command_obj(c)
end

--replace create_dataref
function deferred_dataref(name,type,notifier)
	--print("Deferred dataref: "..name)
	dref=XLuaCreateDataRef(name, type,"yes",notifier)
	return wrap_dref_any(dref,type)
end

--*************************************************************************************--
--**                             CREATE CUSTOM COMMANDS                              **--
--*************************************************************************************--


--*************************************************************************************--
--**                             CREATE CUSTOM DATAREFS                              **--
--*************************************************************************************--
