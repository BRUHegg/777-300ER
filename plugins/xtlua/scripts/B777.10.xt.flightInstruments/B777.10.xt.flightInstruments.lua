--[[
*****************************************************************************************
* Script Name: flightInstruments
* Author Name: Crazytimtimtim
* Script Description: Code for cockpit instruments
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

local B777_kgs_to_lbs = 2.2046226218

--*************************************************************************************--
--**                              FIND X-PLANE DATAREFS                              **--
--*************************************************************************************--

simDR_magHDG                           = find_dataref("sim/cockpit/autopilot/heading_mag")
simDR_trueHDG                          = find_dataref("sim/cockpit/autopilot/heading")

simDR_com1_stby_khz                    = find_dataref("sim/cockpit2/radios/actuators/com1_standby_frequency_khz")
simDR_com1_act_khz                     = find_dataref("sim/cockpit2/radios/actuators/com1_frequency_khz")

simDR_com2_stby_khz                    = find_dataref("sim/cockpit2/radios/actuators/com2_standby_frequency_khz")
simDR_com2_act_khz                     = find_dataref("sim/cockpit2/radios/actuators/com2_frequency_khz")

simDR_total_fuel_kgs                   = find_dataref("sim/flightmodel/weight/m_fuel_total")
simDR_r_fuel_kgs                       = find_dataref("sim/cockpit2/fuel/fuel_level_indicated_right")
simDR_l_fuel_kgs                       = find_dataref("sim/cockpit2/fuel/fuel_level_indicated_left")

simDR_vs_capt                          = find_dataref("sim/cockpit2/gauges/indicators/vvi_fpm_pilot")
simDR_ias_capt                         = find_dataref("sim/cockpit2/gauges/indicators/airspeed_kts_pilot")

--*************************************************************************************--
--**                             CUSTOM DATAREF HANDLERS                             **--
--*************************************************************************************--



--*************************************************************************************--
--**                              CREATE CUSTOM DATAREFS                             **--
--*************************************************************************************--

B777DR_total_fuel_lbs                  = deferred_dataref("Strato/777/displays/total_fuel_lbs", "number")
B777DR_r_fuel_lbs                      = deferred_dataref("Strato/777/displays/r_fuel_lbs", "number")
B777DR_l_fuel_lbs                      = deferred_dataref("Strato/777/displays/l_fuel_lbs", "number")

B777DR_displayed_hdg                   = deferred_dataref("Strato/777/displays/hdg", "number") -- what the MCP heading display actually shows
B777DR_hdg_mode                        = deferred_dataref("Strato/777/displays/hdg_mode", "number")
B777DR_pfd_mtrs_capt                   = deferred_dataref("Strato/777/displays/mtrs_capt", "number")

B777DR_eicas_mode                      = deferred_dataref("Strato/777/displays/eicas_mode", "number") -- what pages the lower eicas is on

B777DR_displayed_com1_act_khz          = deferred_dataref("Strato/777/displays/com1_act_khz", "number") -- COM1 Radio Active Display
B777DR_displayed_com1_stby_khz         = deferred_dataref("Strato/777/displays/com1_stby_khz", "number") -- COM1 Radio Standby Display

B777DR_displayed_com2_act_khz          = deferred_dataref("Strato/777/displays/com2_act_khz", "number") -- COM2 Radio Active Display
B777DR_displayed_com2_stby_khz         = deferred_dataref("Strato/777/displays/com2_stby_khz", "number") -- COM2 Radio Standby Display

B777DR_vs_capt_indicator               = deferred_dataref("Strato/777/displays/vvi_capt", "number")
B777DR_ias_capt_indicator              = deferred_dataref("Strato/777/displays/ias_capt", "number")

--*************************************************************************************--
--**                             X-PLANE COMMAND HANDLERS                            **--
--*************************************************************************************--



--*************************************************************************************--
--**                                 X-PLANE COMMANDS                                **--
--*************************************************************************************--



--*************************************************************************************--
--**                             CUSTOM COMMAND HANDLERS                             **--
--*************************************************************************************--

function B777_mcp_magTRK_CMDhandler(phase, duration)
	if phase == 0 then
		B777DR_hdg_mode = 1 - B777DR_hdg_mode
	end
end

--*************************************************************************************--
--**                             CREATE CUSTOM COMMANDS                               **--
--*************************************************************************************--

B777CMD_mcp_MAGtrk                   = deferred_command("Strato/B777/button_switch/mcp/MAGtrk", "Switch between true and magnetic heading", B777_mcp_magTRK_CMDhandler)


--*************************************************************************************--
--**                                      CODE                                       **--
--*************************************************************************************--

--Clocks

--Lower Eicas

--*************************************************************************************--
--**                                  EVENT CALLBACKS                                **--
--*************************************************************************************--

function aircraft_load()
	print("flightIinstruments loaded")
end

--function aircraft_unload()

function flight_start()
	B777DR_eicas_mode = 4
end

--function flight_crash()

--function before_physics()

function after_physics()
	if B777DR_hdg_mode == 0 then
		B777DR_displayed_hdg = simDR_magHDG
	elseif B777DR_hdg_mode == 1 then
		B777DR_displayed_hdg = simDR_trueHDG
	end

	B777DR_displayed_com1_act_khz = simDR_com1_act_khz / 1000
	B777DR_displayed_com1_stby_khz = simDR_com1_stby_khz / 1000

	B777DR_displayed_com2_act_khz = simDR_com2_act_khz / 1000
	B777DR_displayed_com2_stby_khz = simDR_com2_stby_khz / 1000

	B777DR_total_fuel_lbs = simDR_total_fuel_kgs * B777_kgs_to_lbs
	B777DR_r_fuel_lbs = simDR_r_fuel_kgs * B777_kgs_to_lbs
	B777DR_l_fuel_lbs = simDR_l_fuel_kgs * B777_kgs_to_lbs

	if simDR_vs_capt > 6000 then
		B777DR_vs_capt_indicator = 6000
	elseif simDR_vs_capt < -6000 then
		B777DR_vs_capt_indicator = -6000
	else
		B777DR_vs_capt_indicator = simDR_vs_capt
	end

	if simDR_ias_capt < 30 then
		B777DR_ias_capt_indicator = 30
	elseif simDR_ias_capt > 490 then
		B777DR_ias_capt_indicator = 490
	else
		B777DR_ias_capt_indicator = simDR_ias_capt
	end
end

--function after_replay()