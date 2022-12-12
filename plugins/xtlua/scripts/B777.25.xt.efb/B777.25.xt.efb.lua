--[[
*****************************************************************************************
* Script Name: EFB
* Author Name: crazytimtimtim
* Script Description: Electronic Flight Bag Code
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

local lastpage = 0
local pagemode = 0

--*************************************************************************************--
--**                              FIND X-PLANE DATAREFS                              **--
--*************************************************************************************--



--*************************************************************************************--
--**                             CUSTOM DATAREF HANDLERS                             **--
--*************************************************************************************--



--*************************************************************************************--
--**                              CREATE CUSTOM DATAREFS                             **--
--*************************************************************************************--

B777DR_efb_page = deferred_dataref("Strato/777/displays/efb_page", "number")

--*************************************************************************************--
--**                             X-PLANE COMMAND HANDLERS                            **--
--*************************************************************************************--



--*************************************************************************************--
--**                                 X-PLANE COMMANDS                                **--
--*************************************************************************************--



--*************************************************************************************--
--**                             CUSTOM COMMAND HANDLERS                             **--
--*************************************************************************************--

function efb_L1_CMDhandler(phase, duration)
   if phase == 0 then

   end
end

function efb_L2_CMDhandler(phase, duration)
   if phase == 0 then
      
   end
end

function efb_LC1_CMDhandler(phase, duration)
   if phase == 0 then
      
   end
end

function efb_LC2_CMDhandler(phase, duration)
   if phase == 0 then
      
   end
end

function efb_RC1_CMDhandler(phase, duration)
   if phase == 0 then
      
   end
end

function efb_RC2_CMDhandler(phase, duration)
   if phase == 0 then
      
   end
end

function efb_R1_CMDhandler(phase, duration)
   if phase == 0 then
      
   end
end

function efb_R2_CMDhandler(phase, duration)
   if phase == 0 then
      
   end
end

function efb_home_CMDhandler(phase, duration)
   if phase == 0 then
      B777DR_efb_page = 2
   end
end

function efb_back_CMDhandler(phase, duration)
   if phase == 0 then
      B777DR_efb_page = lastpage
   end
end

function efb_pwr_CMDhandler(phase, duration)
   if phase == 0 then
      if B777DR_efb_page > 0 then
         lastpage = B777DR_efb_page
         B777DR_efb_page = 0
      else
         B777DR_efb_page = 1
         run_after_time(efb_boot, 3)
   end
end

function efb_boot()
   B777DR_efb_page = lastpage
end

--*************************************************************************************--
--**                             CREATE CUSTOM COMMANDS                               **--
--*************************************************************************************--

B777CMD_efb_L1 = deferred_command("Strato/777/efb_L1", "EFB Top Left Button", efb_L1_CMDhandler)
B777CMD_efb_L2 = deferred_command("Strato/777/efb_L2", "EFB Bottom Left Button", efb_L2_CMDhandler)
B777CMD_efb_LC1 = deferred_command("Strato/777/efb_LC1", "EFB Top Left-Center Button", efb_LC1_CMDhandler)
B777CMD_efb_LC2 = deferred_command("Strato/777/efb_LC2", "EFB Bottom Left-Center Button", efb_LC2_CMDhandler)
B777CMD_efb_RC1 = deferred_command("Strato/777/efb_RC1", "EFB Top Right-Center Button", efb_RC1_CMDhandler)
B777CMD_efb_RC2 = deferred_command("Strato/777/efb_RC2", "EFB Bottom Right-Center Button", efb_RC2_CMDhandler)
B777CMD_efb_R1 = deferred_command("Strato/777/efb_R1", "EFB Top Right Button", efb_R1_CMDhandler)
B777CMD_efb_R2 = deferred_command("Strato/777/efb_R2", "EFB Bottom Right Button", efb_R2_CMDhandler)

B777CMD_efb_home = deferred_command("Strato/777/efb_home", "EFB Home Button", efb_home_CMDhandler)
B777CMD_efb_pwr = deferred_command("Strato/777/efb_pwr", "EFB Power Button", efb_pwr_CMDhandler)
B777CMD_efb_back = deferred_command("Strato/777/efb_back", "EFB Back Button", efb_back_CMDhandler)

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
