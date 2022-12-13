--[[
*****************************************************************************************
* Program Script Name	:	B777.05.xt.simconfig
* Author Name			:	Marauder28
*
*   Revisions:
*   -- DATE --	--- REV NO ---		--- DESCRIPTION ---
*   2020-11-19	0.01a				Start of Dev
*
*
*
*
*****************************************************************************************
*
*****************************************************************************************
--]]

--*************************************************************************************--
--** 					         EARLY FUNCTION DEFITIONS                            **--
--*************************************************************************************--
--replace create_command
function deferred_command(name,desc,realFunc)
	return replace_command(name,realFunc)
end
--replace deferred_dataref
function deferred_dataref(name,nilType,callFunction)
if callFunction~=nil then
    print("WARN:" .. name .. " is trying to wrap a function to a dataref -> use xlua")
    end
    return find_dataref(name)
end

dofile("json/json.lua")

simDR_livery_path			= find_dataref("sim/aircraft/view/acf_livery_path")
simDR_acf_tailnum			= find_dataref("sim/aircraft/view/acf_tailnum")

--Baro Sync
simDR_baro_capt				= find_dataref("sim/cockpit/misc/barometer_setting")
simDR_baro_fo				= find_dataref("sim/cockpit/misc/barometer_setting2")

--*************************************************************************************--
--** 				        CREATE READ-WRITE CUSTOM DATAREFS                        **--
--*************************************************************************************--
-- Holds all SimConfig options
B777DR_simconfig_data						= deferred_dataref("laminar/B777/simconfig", "string")
B777DR_newsimconfig_data					= deferred_dataref("laminar/B777/newsimconfig", "number")
B777DR_is_freighter          				= deferred_dataref("laminar/B777/freighter", "number")
B777DR_efis_baro_ref_capt_sel_dial_pos		= find_dataref("laminar/B777/efis/baro_ref/capt/sel_dial_pos", "number")
B777DR_efis_baro_ref_fo_sel_dial_pos		= deferred_dataref("laminar/B777/efis/baro_ref/fo/sel_dial_pos", "number")
B777DR_flt_inst_inbd_disp_capt_sel_dial_pos	= deferred_dataref("laminar/B777/flt_inst/capt_inbd_display/sel_dial_pos", "number")
B777DR_flt_inst_lwr_disp_capt_sel_dial_pos	= deferred_dataref("laminar/B777/flt_inst/capt_lwr_display/sel_dial_pos", "number")
B777DR_flt_inst_inbd_disp_fo_sel_dial_pos	= deferred_dataref("laminar/B777/flt_inst/fo_inbd_display/sel_dial_pos", "number")
B777DR_flt_inst_lwr_disp_fo_sel_dial_pos	= deferred_dataref("laminar/B777/flt_inst/fo_lwr_display/sel_dial_pos", "number")
B777DR_pfd_style							= deferred_dataref("laminar/B777/pfd/style", "number")
B777DR_nd_style								= deferred_dataref("laminar/B777/nd/style", "number")
B777DR_thrust_ref							= deferred_dataref("laminar/B777/engines/thrust_ref", "number")

--Baro Sync
B777DR_efis_baro_ref_capt_sel_dial_pos		= deferred_dataref("laminar/B777/efis/baro_ref/capt/sel_dial_pos", "number")
B777DR_efis_baro_ref_fo_sel_dial_pos		= deferred_dataref("laminar/B777/efis/baro_ref/fo/sel_dial_pos", "number")
B777DR_efis_baro_ref_capt_switch_pos		= deferred_dataref("laminar/B777/efis/baro_std/capt/switch_pos", "number")
B777DR_efis_baro_ref_fo_switch_pos			= deferred_dataref("laminar/B777/efis/baro_std/fo/switch_pos", "number")

-- Engine Type (crazytimitmtim)
B777DR_engineType                               = deferred_dataref("laminar/B777/engines/type", "number")
B777DR_hideGE						= deferred_dataref("laminar/B777/engines/hideGE", "number") 
B777DR_hideRR						= deferred_dataref("laminar/B777/engines/hideRR", "number") 
B777DR_SNDoptions			        	= deferred_dataref("laminar/B777/fmod/options", "array[7]")
--B777DR_SNDoptions_volume				= deferred_dataref("laminar/B777/fmod/options/volume", "array[8]")
B777DR_SNDoptions_gpws					= deferred_dataref("laminar/B777/fmod/options/gpws", "array[16]")
B777DR_modernAlarms						= deferred_dataref("laminar/B777/fmod/options/modernAlarms", "number")
--*************************************************************************************--
--** 				        MAIN PROGRAM LOGIC                                       **--
--*************************************************************************************--
simConfigData = {}
dofile("version.lua")
--[[function simconfig_values()
	return {
			SIM = {
					 kgs_to_lbs = 2.2046226218488,
					 lbs_to_kgs = 0.45359237,
					 weight_display_units = "KGS",  --KGS, LBS
					 std_pax_weight = 120.0,  --KGS
					 baro_indicator = "IN",  --IN = 0, HPA = 1
					 baro_sync = "NO",  --Sync to CAPT
					 capt_inbd = "NORM",  --EICAS = 0, NORM = 1, PFD = 2
					 capt_lwr = "NORM",  --EICAS PRI = 0, NORM = 1, ND = 2
					 fo_inbd = "NORM",  --PFD = 0, NORM = 1, EICAS = 2
					 fo_lwr = "NORM",  --ND = 0, NORM = 1, EICAS PRI = 2
			},
			},
			FMC = {
					  INIT = {
								nav_data = "",
								active = "",
								op_program = fmcVersion,
								drag_ff = "+0.0/-0.0",
					  },
			},
			SOUND = { --default 0 on
				paOption = 0,
				musicOption = 0,
				PM_toggle = 0,
				V1Option = 0,
				GPWSminimums = 0,
				GPWSapproachingMinimums = 0,
				GPWS2500 = 0,
				GPWS1000 =0,
				GPWS500 = 0,
				GPWS400 = 0,
				GPWS300 = 0,
				GPWS200 = 0,
				GPWS100 = 0,
				GPWS50 = 0,
				GPWS40 = 0,
				GPWS30 = 0,
				GPWS20 = 0,
				GPWS10 = 0,
				GPWS5 = 0
			}
	}
end]]

function simconfig_values()
	return {
		SOUND = {
			paOption = 1,
			musicOption = 1,
			PM_toggle = 1,
			V1Option = 1,
			GPWSminimums = 1,
			GPWSapproachingMinimums = 1,
			GPWS2500 = 1,
			GPWS1000 = 1,
			GPWS500 = 1,
			GPWS400 = 1,
			GPWS300 = 1,
			GPWS200 = 1,
			GPWS100 = 1,
			GPWS50 = 1,
			GPWS40 = 1,
			GPWS30 = 1,
			GPWS20 = 1,
			GPWS10 = 1,
			GPWS5 = 1,
		},
		PLANE = {
			aircraft_type = "PASSENGER", --Passenger, Freighter
			airline = "",
		},
		FMC = {
			nav_data = "",
			active = "",
			op_program = fmcVersion,
			drag_ff = "+0.0/-0.0",
		},
		OPTIONS = {
			weight_display_units = "LBS",
			iru_align_real = 1,
			iru_align_time = 0, -- if manual
			real_park_brake = 1,
			trs_bug = 1,
			aoa_indicator = 1,
			smart_knobs = 1,
			press_mins_sync = 0 -- 0 = no sync, 1 = sync to capt, 2 = sync to fo
		}
	}
end

function baro_sync()
	if simConfigData["data"].SIM.baro_sync == "YES" then
		simDR_baro_fo = simDR_baro_capt
		B777DR_efis_baro_ref_fo_sel_dial_pos = B777DR_efis_baro_ref_capt_sel_dial_pos
		B777DR_efis_baro_ref_fo_switch_pos = B777DR_efis_baro_ref_capt_switch_pos
	end
end

-- crazytimtimtim end
function setSoundOption(key,value)

	if key == "alarmsOption" then
		B777DR_SNDoptions[0] = value
	end

	if key == "seatBeltOption" then
		B777DR_SNDoptions[1] = value
		return
	end

	if key == "paOption" then 
		B777DR_SNDoptions[2] = value
		return
	end

	if key == "musicOption" then 
		B777DR_SNDoptions[3] = value
		return 
	end

	if key == "PM_toggle" then 
		B777DR_SNDoptions[4] = value
		return
	end

	if key == "V1Option" then B777DR_SNDoptions[5] = value return end
	if key == "GPWSminimums" then B777DR_SNDoptions_gpws[1] = value return end
	if key == "GPWSapproachingMinimums" then B777DR_SNDoptions_gpws[2] = value return end
	if key == "GPWS2500" then B777DR_SNDoptions_gpws[3] = value return end
	if key == "GPWS1000" then B777DR_SNDoptions_gpws[4] = value return end
	if key == "GPWS500" then B777DR_SNDoptions_gpws[5] = value return end
	if key == "GPWS400" then B777DR_SNDoptions_gpws[6] = value return end
	if key == "GPWS300" then B777DR_SNDoptions_gpws[7] = value return end
	if key == "GPWS200" then B777DR_SNDoptions_gpws[8] = value return end
	if key == "GPWS100" then B777DR_SNDoptions_gpws[9] = value return end
	if key == "GPWS50" then B777DR_SNDoptions_gpws[10] = value return end
	if key == "GPWS40" then B777DR_SNDoptions_gpws[11] = value return end
	if key == "GPWS30" then B777DR_SNDoptions_gpws[12] = value return end
	if key == "GPWS20" then B777DR_SNDoptions_gpws[13] = value return end
	if key == "GPWS10" then B777DR_SNDoptions_gpws[14] = value return end
	if key == "GPWS5" then B777DR_SNDoptions_gpws[15] = value return end

end

function set_loaded_configs()

	for key, value in pairs(simConfigData["data"].SOUND) do
		setSoundOption(key,value)
	end
	if simConfigData["data"].PLANE.aircraft_type ~= "PASSENGER" then
		B777DR_is_freighter= 1
	else
		B777DR_is_freighter= 0
	end
	print("B777DR_is_freighter="..B777DR_is_freighter)
	B777DR_newsimconfig_data=0

end

function aircraft_simConfig()

	local file_location = simDR_livery_path.."B777-400_simconfig.dat"
	print("File = "..file_location)
	local file = io.open(file_location, "r")

	if file ~= nil then
		io.input(file)
		local tmpDataS = io.read()
		io.close(file)
		--print("read "..tmpDataS)
		local tmpData=json.decode(tmpDataS)
		if (tmpData["SOUND"]==nil) then
			tmpData["SOUND"]=simconfig_values()["SOUND"]
		end
		tmpData.FMC.INIT.op_program = fmcVersion
		--print("encoding "..tmpDataS)
		B777DR_simconfig_data = json.encode(tmpData)
		--print("done encoding "..B777DR_simconfig_data)
		B777DR_newsimconfig_data=1
		run_after_time(set_loaded_configs, 3)  --Apply loaded configs.  Wait a few seconds to ensure they load correctly.
	else
		B777DR_simconfig_data = json.encode(simconfig_values())
		run_after_time(set_loaded_configs, 3)  --Apply loaded configs.  Wait a few seconds to ensure they load correctly.
	end

end

simConfigData["data"]=simconfig_values()

if simDR_acf_tailnum ~= nil then
	simConfigData["data"].PLANE.fin_nbr = simDR_acf_tailnum
end

function flight_start()
	local refreshLivery=simDR_livery_path
	B777DR_simconfig_data=json.encode(simConfigData["data"]["values"]) --make the simConfig data available to other modules
	B777DR_newsimconfig_data=1
	run_after_time(aircraft_simConfig, 1)  --Load specific simConfig data for current livery
end
function livery_load()
	--print("simcongfig livery_load")
	local refreshLivery=simDR_livery_path
	B777DR_newsimconfig_data=1
	run_after_time(aircraft_simConfig, 2)  --Load specific simConfig data for current livery
end

local setSimConfig=false
function hasSimConfig()
	if B777DR_newsimconfig_data==1 then
		if string.len(B777DR_simconfig_data) > 1 then
			simConfigData["data"] = json.decode(B777DR_simconfig_data)
			setSimConfig=true
		else
			return false
		end
	end
	return setSimConfig
end
function after_physics()
	--Keep the structure fresh
	if hasSimConfig()==false then return end


	--See if Baro's should be sync'd
	baro_sync()
	
	--PFD/ND Styles
	check_pfd_nd_style()
	
	--Thrust Ref Setting
	check_thrust_ref()

	--Engine Type
	checkEngineType()
end
