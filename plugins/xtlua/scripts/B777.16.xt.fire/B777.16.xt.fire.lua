--[[
*****************************************************************************************
* Script Name: Fire
* Author Name: nathroxer001
* Script Description: Fire system
*****************************************************************************************
--]]

--replace create_command
function deferred_command(name,desc,realFunc)
    return replace_command(name,realFunc)
 end
 
 --replace create_dataref
 function deferred_dataref(name,nilType,callFunction)
    if callFunction~=nil then
       print("WARN:" .. name .. " is trying to wrap a function to a dataref -> use xlua")
    end
    return find_dataref(name)
 end
 
 
 --*************************************************************************************--
 --**                             XTLUA GLOBAL VARIABLES                              **--
 --*************************************************************************************--
 
 --[[
 SIM_PERIOD - this contains the duration of the current frame in seconds (so it is alway a
 fraction).  Use this to normalize rates,  e.g. to add 3 units of fuel per second in a
 per-frame callback you’d do fuel = fuel + 3 * SIM_PERIOD.
 
 IN_REPLAY - evaluates to 0 if replay is off, 1 if replay mode is on
 --]]
 
 --*************************************************************************************--
 --**                                CREATE VARIABLES                                 **--
 --*************************************************************************************--
 
 
 
 --*************************************************************************************--
 --**                              FIND X-PLANE DATAREFS                              **--
 --*************************************************************************************--
 
 
 
 --*************************************************************************************--
 --**                             CUSTOM DATAREF HANDLERS                             **--
 --*************************************************************************************--
 
 function B777DR_fire_ovhd_test_DR_handler(phase, duration)
    if phase == 0 then
        B777DR_fire_ovhd_test = 1
    else if phase == 1 or 2 then
        B777DR_fire_ovhd_test = 0 
    end
    end

end
 
 --*************************************************************************************--
 --**                              CREATE CUSTOM DATAREFS                             **--
 --*************************************************************************************--
 
 B777DR_fire_ovhd_test = deferred_dataref("Strato/777/cockpit/cockpit_ovhd_fire_test","number")
 
 --*************************************************************************************--
 --**                             X-PLANE COMMAND HANDLERS                            **--
 --*************************************************************************************--
 
 
 
 --*************************************************************************************--
 --**                                 X-PLANE COMMANDS                                **--
 --*************************************************************************************--
 
 
 
 --*************************************************************************************--
 --**                             CUSTOM COMMAND HANDLERS                             **--
 --*************************************************************************************--
 
 
 
 --*************************************************************************************--
 --**                             CREATE CUSTOM COMMANDS                               **--
 --*************************************************************************************--
 
 
 
 --*************************************************************************************--
 --**                                      CODE                                       **--
 --*************************************************************************************--
 

 
 --*************************************************************************************--
 --**                                  EVENT CALLBACKS                                **--
 --*************************************************************************************--
 
 --function aircraft_load()
 
 --function aircraft_unload()
 
 --function flight_start()
 
 --function flight_crash()
 
 --function before_physics()
 
 --function after_physics()
 
 --function after_replay()
 
 