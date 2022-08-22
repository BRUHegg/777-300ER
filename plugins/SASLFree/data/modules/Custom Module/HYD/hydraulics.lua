--[[
*****************************************************************************************
* Script Name: Hydraulic
* Author Name: @bruh
* Script Description: Code for Hydraulic systems.
*****************************************************************************************
--]]

addSearchPath(moduleDirectory .. "/Custom Module/HYD/graphics")
addSearchPath(moduleDirectory .. "/Custom Module/")

include("misc_tools.lua")

--Finding sim datarefs

--Cockpit controls
battery = globalPropertyiae("sim/cockpit2/electrical/battery_on", 1)
speedbrake_handle = globalPropertyf("sim/cockpit2/controls/speedbrake_ratio")
--Speeds
ias = globalPropertyf("sim/flightmodel/position/indicated_airspeed2")
tas = globalPropertyf("sim/flightmodel/position/true_airspeed") --true airspeed in meters per second. Needed for accurate drooping
--Indicators
ra_pilot = globalPropertyf("sim/cockpit2/gauges/indicators/radio_altimeter_height_ft_pilot")
ra_copilot = globalPropertyf("sim/cockpit2/gauges/indicators/radio_altimeter_height_ft_copilot")
--Weather and position
oat = globalPropertyf("sim/cockpit2/temperature/outside_air_temp_degc")
t_phi = globalPropertyf("sim/flightmodel/position/true_phi") --bank angle
--Flight controls
flaps = globalPropertyfae("sim/flightmodel2/wing/flap1_deg", 1)
f_time = globalPropertyf("sim/operation/misc/frame_rate_period")
--Engines
engine_1_n2 = globalPropertyfae("sim/flightmodel/engine/ENGN_N2_", 1)
engine_2_n2 = globalPropertyfae("sim/flightmodel/engine/ENGN_N2_", 2)
--Ground collision
on_ground = globalPropertyi("sim/flightmodel/failures/onground_any")

--Finding own datarefs

c_time = globalPropertyf("Strato/777/time/current")
act_press = globalPropertyi("Strato/777/gear/actuator_press")

--Creating datarefs

hyd_qty = createGlobalPropertyfa("Strato/777/hydraulics/qty", {0.98, 0.95, 0.97})
system_load = createGlobalPropertyfa("Strato/777/hydraulics/sys_load", {0, 0, 0}) --hydraulic load per every system
hyd_pressure = createGlobalPropertyia("Strato/777/hydraulics/press", {50, 50, 50}) --this dataref is shared bc we need it to be readable from xtlua smh
demand_pumps_state = createGlobalPropertyia("Strato/777/hydraulics/pump/demand/state", {0, 0, 0, 0})
demand_pumps_actual = createGlobalPropertyia("Strato/777/hydraulics/pump/demand/actual", {0, 0, 0, 0})
demand_pumps_past = createGlobalPropertyia("Strato/777/hydraulics/pump/demand/past", {0, 0, 0, 0}) --this is needed for accurate schematic update on the hydraulic page of eicas since it happens with a 2/3 second delay irl
hyd_temps_demand = createGlobalPropertyfa("Strato/777/hydraulics/pump/demand/temperatures", {0, 0, 0, 0})
hyd_fail_demand = createGlobalPropertyia("Strato/777/hydraulics/pump/demand/fail", {0, 0, 0, 0})
demand_fault = createGlobalPropertyia("Strato/777/hydraulics/pump/demand/fault", {0, 0, 0, 0}) --status of the light itself
demand_fault_physical = createGlobalPropertyfa("Strato/777/hydraulics/pump/demand/fault_physical", {0, 0, 0, 0})
demand_fault_tgt = createGlobalPropertyia("Strato/777/hydraulics/pump/demand/fault_tgt", {0, 0, 0, 0})
primary_pumps_state = createGlobalPropertyia("Strato/777/hydraulics/pump/primary/state", {1, 0, 0, 1})
primary_pumps_actual = createGlobalPropertyia("Strato/777/hydraulics/pump/primary/actual", {0, 0, 0, 0})
primary_pumps_past = createGlobalPropertyia("Strato/777/hydraulics/pump/primary/past", {0, 0, 0, 0})
hyd_temps_primary = createGlobalPropertyfa("Strato/777/hydraulics/pump/primary/temperatures", {0, 0, 0, 0})
hyd_fail_primary = createGlobalPropertyia("Strato/777/hydraulics/pump/primary/fail", {0, 0, 0, 0})
primary_fault = createGlobalPropertyia("Strato/777/hydraulics/pump/primary/fault", {0, 0, 0, 0})

tmp = 0 --For updating hydraulic pressure
t = tmp --For EICAS
t1 = tmp --For updating temperatures

--Fault light logic
--Primary pumps
primary_past  = {0, 0, 0, 0}
primary_shutdown = {0, 0, 0, 0}
primary_ovht = {0, 0, 0, 0}

--Demand pumps
demand_ovht = {0, 0, 0, 0}
demand_time = {0, 0, 0, 0}
demand_shutdown = {0, 0, 0, 0}
demand_was_working = {0, 0, 0, 0} --Was the demand pump working at time of shut down?

--Left ACMP low pressure logic
L_ACMP_time = -16 -- time when pressure reached 2400 psi with both pumps on
L_ACMP_engage = false --engage due to low pressure

--ADP touchdown logic
ADP_speedbrake_retract = -6
ADP_touchdown = false
--ADP low pressure logic
ADP_time = -16
ADP_engage = false
--
ADP_onground_past = 0
ADP_leave_ground_time = -16
--Other ADP logic
ADP_flap_past = 0
ADP_flap_pump = 2

--Right ACMP low pressure logic
R_ACMP_time = -16 -- time when pressure reached 2400 psi with both pumps on
R_ACMP_engage = false

temp_acc = {0, 0, 0}
hyd_qty_initial = {0.98, 0.95, 0.97}

function UpdateLights()
	for i=1,4 do
		--DRefs for primary pumps
		local light_primary = globalPropertyiae("Strato/777/hydraulics/pump/primary/fault", i)
		local primary_switch = globalPropertyiae("Strato/777/hydraulics/pump/primary/state", i)
		local primary_temperature = globalPropertyfae("Strato/777/hydraulics/pump/primary/temperatures", i)
		--Low pressure logic
		if get(primary_switch) ~= primary_past[i] then
			if get(primary_switch) == 0 then
				primary_shutdown[i] = get(c_time)
			end
			primary_past[i] = get(primary_switch)
		end
		--Overheat logic
		if get(primary_temperature) >= 75 and primary_ovht[i] == 0 then
			primary_shutdown[i] = get(c_time)
			primary_ovht[i] = 1
		elseif get(primary_temperature) < 75 then
			primary_ovht[i] = 0
		end
		if get(primary_switch) == 0 and get(c_time) >= primary_shutdown[i] + 5 then
			set(light_primary, 1)
		elseif get(c_time) >= primary_shutdown[i] + 5 and get(primary_temperature) >= 75 then
			set(light_primary, 1)
		else
			set(light_primary, 0)
		end
		--DRefs for demand pumps
		local light_demand = globalPropertyiae("Strato/777/hydraulics/pump/demand/fault", i)
		local demand_switch = globalPropertyiae("Strato/777/hydraulics/pump/demand/state", i)
		local demand_past = globalPropertyiae("Strato/777/hydraulics/pump/demand/past", i)
		local demand_temperature = globalPropertyfae("Strato/777/hydraulics/pump/demand/temperatures", i)
		--Wait 5 seconds if a working demand pump was shut down
		if get(demand_past) == 1 and get(demand_switch) == 0 and demand_was_working[i] == 0 then
			demand_shutdown[i] = get(c_time)
			demand_was_working[i] = 1
		elseif get(demand_past) == 0 and get(demand_switch) == 0 and demand_was_working[i] == 0 then
			demand_shutdown[i] = get(c_time) - 5
		end
		--Demand pump overheat logic
		if get(demand_temperature) >= 75 and demand_ovht[i] == 0 then
			demand_ovht[i] = 1
			demand_time[i] = get(c_time)
		elseif get(demand_temperature) < 75 then
			demand_ovht[i] = 0
		end
		if (get(c_time) >= demand_shutdown[i] + 5 and get(demand_switch) == 0) or (get(c_time) >= demand_time[i] + 5 and demand_ovht[i] == 1) then
			set(light_demand, 1)
		else
			set(light_demand, 0)
		end
	end
end

function GetMaxHydPress() --since some things are driven by all 3 hydraulic systems, we need to know the maximum pressure
	local pressure_L = globalPropertyiae("Strato/777/hydraulics/press", 1)
	local pressure_C = globalPropertyiae("Strato/777/hydraulics/press", 2)
	local pressure_R = globalPropertyiae("Strato/777/hydraulics/press", 3)
	return math.max(get(pressure_L), get(pressure_C), get(pressure_R))
end

function GetPrimaryPumpState(idx) --this function defines primary pump logic.
	local pump_state = globalPropertyiae("Strato/777/hydraulics/pump/primary/state", idx)
	local apu_running = globalPropertyi("sim/cockpit2/electrical/APU_running")
	local fault = globalPropertyiae("Strato/777/hydraulics/pump/primary/fail", idx) 
	if get(fault) == 1 then
		return -1
	end
	if idx == 1 then
		if get(pump_state) == 1 and get(engine_1_n2) >= 10 then
			return 1
		end
	elseif idx == 2 then
		if IsAcConnected() == 1 and get(pump_state) == 1 then
			return 1
		end
	elseif idx == 3 then 
		if get(globalPropertyi("sim/flightmodel/failures/onground_any")) == 1 and get(pump_state) == 1 then
			if get(engine_1_n2) < 18 and get(engine_2_n2) < 18 then
				if get(apu_running) == 1 and IsAcConnected() == 1 then
					return 1
				end
			elseif get(engine_1_n2) > 18 and get(engine_2_n2) > 18 and IsAcConnected() == 1 then --C2 pump only activates when there are 2 power sources
				return 1
			end
		elseif get(on_ground) == 0 and get(pump_state) == 1 then
			if IsAcConnected() == 1 then
				return 1
			end
		end
	else
		if get(pump_state) == 1 and get(engine_2_n2) >= 10 then
			return 1
		end
	end
	return 0
end

function GetDemandPumpState(idx, primary_state)
	local fault = globalPropertyiae("Strato/777/hydraulics/pump/demand/fail", idx) 
	if get(fault) == 1 then
		return -1
	end
	if IsAcConnected() == 1 then
		local demand_state = globalPropertyiae("Strato/777/hydraulics/pump/demand/state", idx)
		local demand_2 = globalPropertyiae("Strato/777/hydraulics/pump/demand/state", 2)
		if get(demand_state) == 2 then --if the pump is on, it's on
			if idx ~= 3 then 
				return 1
			else
				if get(demand_2) ~= 2 then --C1 and C2 pumps can't work together for continuous amount of time
					return 1
				end
			end
		elseif get(demand_state) == 1 then --auto mode logic
			local sys_idx = GetSysIdx(idx)
			local pressure = globalPropertyiae("Strato/777/hydraulics/press", sys_idx)
			local h_load = globalPropertyfae("Strato/777/hydraulics/sys_load", sys_idx)
			local flt_state = GetFltState()
			if flt_state == 1 and idx == 2 or flt_state == 1 and idx == 3 then --turn ADPs on if we are taking off
				return 1
			else
				if idx == 1 then
					if get(ias) > 60 and get(ra_pilot) <= 30 or get(ias) > 60 and get(ra_copilot) <= 30 then --Getting ready for deploying them speedbrakes
						return 1
					elseif get(pressure) < 2800 then --Low pressure logic
						if L_ACMP_engage == false then
							L_ACMP_engage = true
						end
						return 1
					elseif L_ACMP_engage == true and get(pressure) >= 2800 then
						if primary_state == 1 then
							L_ACMP_time = get(c_time)
							L_ACMP_engage = false
						end
						return 1
					elseif L_ACMP_engage == false and get(pressure) >= 2800 and get(c_time) < L_ACMP_time + 15 then
						return 1
					end
				elseif idx == 2 or idx == 3 then
					--Acquiring state of the second primary pump in the system
					local primary_state_second = 0
					if idx == 2 then
						primary_state_second = globalPropertyiae("Strato/777/hydraulics/pump/primary/actual", 2)
					else
						primary_state_second = globalPropertyiae("Strato/777/hydraulics/pump/primary/actual", 3)
					end
					--Acquiring state of the second demand pump in the system
					local demand_second = 0
					if idx == 2 then
						demand_second = globalPropertyiae("Strato/777/hydraulics/pump/demand/state", 3)
					else
						demand_second = globalPropertyiae("Strato/777/hydraulics/pump/demand/state", 2)
					end
					--Updating touchdown logic for air driven demand pumps
					if get(ias) > 80 and get(ra_pilot) <= 30 or get(ias) > 80 and get(ra_copilot) <= 30 then
						ADP_touchdown = true
					elseif get(ias) <= 80 and ADP_touchdown == true then
						if get(speedbrake_handle) <= 0 then
							ADP_touchdown = false
							ADP_speedbrake_retract = get(c_time)
						end
					elseif ADP_touchdown == true and get(ra_pilot) >= 30 or ADP_touchdown == true and get(ra_copilot) >= 30 then --if we've gone around, set touchdown to false
						ADP_touchdown = false
					end
					if idx == 2 then
						--Updating ADP low pressure logic
						if get(pressure) < 2400 then
							ADP_engage = true
						elseif ADP_engage == true and get(pressure) >= 2700 then
							if primary_state == 1 and get(primary_state_second) == 1 then
								ADP_time = get(c_time)
								ADP_engage = false
							end
						end
						--ADP low pressure logic
						if ADP_engage == false and get(pressure) >= 2700 and get(c_time) < ADP_time + 15 or ADP_engage == true then --Keep pump working until 15 seconds pass from the moment when pressure has reached 2700 psi
							return 1
						end
					end
					--Takeoff logic
					if get(on_ground) ~= ADP_onground_past then
						if get(on_ground) == 0 and ADP_onground_past == 1 then
							ADP_leave_ground_time = get(c_time) --saving time we left the ground gor correct logic
						end
						ADP_onground_past = get(on_ground)
					end
					if get(act_press) > 0 then
						return 1
					end
					local flap_tgt = globalPropertyf("Strato/777/flaps/tgt")
					--ADP flap movement logic
					if get(flap_tgt) == math.abs(Round(get(flaps), 2)) and ADP_flap_past ~= math.abs(Round(get(flaps), 2)) then
						ADP_flap_past = math.abs(Round(get(flaps), 2))
						if ADP_flap_pump == 2 then
							ADP_flap_pump = 3
						else
							ADP_flap_pump = 2
						end
					end
					if get(flap_tgt) ~= math.abs(Round(get(flaps), 2)) then --engage adequate ADP when extending flaps
						if idx == ADP_flap_pump then
							return 1
						end
						ADP_flap_past = math.abs(Round(get(flaps), 2))
					end
					--After takeoff logic
					if get(c_time) < ADP_leave_ground_time + 10 then
						return 1
					end
					if get(on_ground) == 0 and get(flaps) > 0 and get(demand_second) == 0 then --preparing for some autoslat demand
						return 1
					end
					--ADP after landing logic
					if ADP_touchdown == true or get(c_time) < ADP_speedbrake_retract + 5 then --Engage during landing and when the pressure is low 
						return 1
					end
				elseif idx == 4 then
					if get(ra_pilot) <= 30 or get(ra_copilot) <= 30 then --preparing for brake demand if rto happens
						return 1
					elseif get(pressure) < 2800 then --low pressure logic
						if R_ACMP_engage == false then
							R_ACMP_engage = true
						end
						return 1
					elseif R_ACMP_engage == true and get(pressure) >= 2800 then
						if primary_state == 1 then
							R_ACMP_time = get(c_time)
							R_ACMP_engage = false
						end
						return 1
					elseif R_ACMP_engage == false and get(pressure) >= 2800 and get(c_time) < R_ACMP_time + 15 then
						return 1
					end
				end
			end
		end
	end
	return 0
end

function UpdateActualState()
	for i=1,4 do
		local primary_actual = globalPropertyiae("Strato/777/hydraulics/pump/primary/actual", i)
		local demand_actual = globalPropertyiae("Strato/777/hydraulics/pump/demand/actual", i)
		local primary_state = GetPrimaryPumpState(i)
		set(primary_actual, primary_state)
		set(demand_actual, GetDemandPumpState(i, primary_state))
	end
end

function GetTotalPumpsWorking()
	local num = 0
	for i=1,4 do
		local primary_state = globalPropertyiae("Strato/777/hydraulics/pump/primary/actual", i)
		local demand_state = globalPropertyiae("Strato/777/hydraulics/pump/demand/actual", i)
		if get(primary_state) == 1 then
			num = num + 1
		end
		if get(demand_state) == 1 then
			num = num + 1
		end
	end
	return num
end

function GetNWorkingHydPumps(sys_num)
	pumps = {{1}, {2, 3}, {4}}
	n_pumps = {1, 2, 1}
	local num = 0
	for i=1,n_pumps[sys_num] do
		local primary_state = globalPropertyiae("Strato/777/hydraulics/pump/primary/actual", pumps[sys_num][i])
		local demand_state = globalPropertyiae("Strato/777/hydraulics/pump/demand/actual", pumps[sys_num][i])
		if get(primary_state) == 1 then
			num = num + 1
		end
		if get(demand_state) == 1 then
			num = num + 1
		end
	end
	return num
end

function UpdateSysLoad()
	local sys_operative = {} --All indexes of systems that have at least 1 pump working go here
	local n_sys_operative = 0
	for i=1,3 do
		if GetNWorkingHydPumps(i) >= 1 then
			table.insert(sys_operative, n_sys_operative + 1, i)
			n_sys_operative = n_sys_operative + 1
		end
	end
	if n_sys_operative > 0 then
		local load_all = 0
		for i=1,6 do
			local h_load = globalPropertyfae("Strato/777/hydraulics/load", i)
			load_all = load_all + get(h_load)
		end
		for i=1,n_sys_operative do
			local load_c = 0
			local load_l = 0
			local load_r = 0
			local h_load = globalPropertyfae("Strato/777/hydraulics/sys_load", sys_operative[i])
			if sys_operative[i] == 1 then
				local h_load_l = globalPropertyfae("Strato/777/hydraulics/load", 10)
				load_l = get(h_load_l)
			elseif sys_operative[i] == 2 then
				for i=7,9 do
					local h_load_c = globalPropertyfae("Strato/777/hydraulics/load", i)
					load_c = load_c + get(h_load_c)
				end
				local brake_load = globalPropertyfae("Strato/777/gear/brake_load", sys_operative[i] - 1)
				load_c = load_c + get(brake_load) * 0.1
			elseif sys_operative[i] == 3 then
				local brake_load = globalPropertyfae("Strato/777/gear/brake_load", sys_operative[i] - 1)
				local h_load_r = globalPropertyfae("Strato/777/hydraulics/load", 11)
				load_r = get(h_load_r) + get(brake_load) * 0.3
			end
			set(h_load, Round(load_all / n_sys_operative, 2) + load_c + load_r + load_l)
		end
	end
end

function UpdateTemperatures(delay)
	if t1 + delay >= get(c_time) then
		local fuel_reqd = 50 --fuel required for sufficient cooling
		if get(oat) > 0 then
			fuel_reqd = 500 * (get(oat) / 15)
		end
		local pumps_on_total = GetTotalPumpsWorking()
		for i=1,4 do
			local primary_fail = globalPropertyiae("Strato/777/hydraulics/pump/primary/fail", i) 
			local demand_fail = globalPropertyiae("Strato/777/hydraulics/pump/demand/fail", i) 
			--pump temperatures, states
			local primary_temp = globalPropertyfae("Strato/777/hydraulics/pump/primary/temperatures", i) 
			local demand_temp = globalPropertyfae("Strato/777/hydraulics/pump/demand/temperatures", i)
			local primary_state = globalPropertyiae("Strato/777/hydraulics/pump/primary/actual", i)
			local demand_state = globalPropertyiae("Strato/777/hydraulics/pump/demand/actual", i)
			--hydarulic system related things
			local sys_idx = GetSysIdx(i)
			local tk_weight = globalPropertyfae("sim/flightmodel/weight/m_fuel", sys_idx) --weight of the fuel tank used to cool the system
			local h_load = globalPropertyfae("Strato/777/hydraulics/sys_load", sys_idx)
			local pumps_on = GetNWorkingHydPumps(sys_idx)
			local t_acc = temp_acc[sys_idx]
			if t_acc > 10 and pumps_on == 0 then
				t_acc = t_acc - 0.4
			end
			if get(tk_weight) < fuel_reqd then
				local pump_load = 0
				if pumps_on_total > 0 then
					pump_load = get(h_load) / pumps_on_total
				end
				if get(oat) > 5 then
					if t_acc < 12.5 * pumps_on then --Updating accumulated temperature
						t_acc = t_acc + 2 * pump_load * pumps_on
					elseif get(t_acc) > 12.5 * pumps_on then
						t_acc = t_acc - 0.4
					end
				else
					if t_acc > 5 * pumps_on then
						t_acc = t_acc - 0.4 --if oat is too small, decrease accumulated temperature
					end
				end
				if get(primary_fail) == 0 then
					local tgt_temp_p = get(oat) + t_acc + 100 * pump_load * get(primary_state)
					set(primary_temp, get(primary_temp) + (tgt_temp_p - get(primary_temp)) * 0.4)
					if get(primary_temp) > 110 then
						set(primary_fail, 1)
					end
				end
				if get(demand_fail) == 0 then
					local tgt_temp_d = get(oat) + get(t_acc) + 100 * pump_load * get(demand_state)
					set(demand_temp, get(demand_temp) + (tgt_temp_d - get(demand_temp)) * 0.4)
					if get(demand_temp) > 110 then
						set(demand_fail, 1)
					end
				end
			else
				set(primary_temp, get(primary_temp) + (get(oat) * 0.9 - get(primary_temp)) * 0.4)
				set(demand_temp, get(demand_temp) + (get(oat) * 0.9 - get(demand_temp)) * 0.4)
			end
		end
		t1 = get(c_time)
	end
end

function UpdatePressure(delay) --Updates hydraulic pressure based on quantity, working pumps, etc
	if tmp + delay <= get(c_time) then
		for i=1,4 do
			local pumps_on = 0
			local sys_idx = GetSysIdx(i)
			local pressure = globalPropertyiae("Strato/777/hydraulics/press", sys_idx)
			local quantity = globalPropertyfae("Strato/777/hydraulics/qty", sys_idx)
			local pumps_on = GetNWorkingHydPumps(sys_idx)
			if pumps_on > 0 and round(get(t_phi) / 180) % 2 == 0 then
				local primary_temp = globalPropertyfae("Strato/777/hydraulics/pump/primary/temperatures", i) 
				local demand_temp = globalPropertyfae("Strato/777/hydraulics/pump/demand/temperatures", i)
				local primary_fail = globalPropertyiae("Strato/777/hydraulics/pump/primary/fail", i) 
				local demand_fail = globalPropertyiae("Strato/777/hydraulics/pump/demand/fail", i) 
				local desired_pressure = 0
				local increase = 0
				local load_total = globalPropertyfae("Strato/777/hydraulics/sys_load", sys_idx)
				local decrease = 400 * get(load_total) + bool2num(i ~= 2) * round(math.random(-1, 1)) * 10 --decrease due to performance degradation when overheating and due to change in load
				if i ~= 2 then
					l_past = get(load_total)
				end
				local total_pumps = GetTotalPumpsWorking()
				if get(primary_temp) >= 75 and get(primary_fail) == 0 or get(demand_temp) >= 75 and get(demand_fail) == 0 then
					decrease = decrease + 400 / pumps_on
				end
				if get(pressure) < 2700 then
					increase = 80 * (1 - get(load_total) / total_pumps) --pressure increase would vary depending on load
				else
					increase = 20 * (1 - get(load_total) / total_pumps)
				end
				if i == 2 or i == 3 then
					desired_pressure = (3000 + 50 * (pumps_on - 1)) * round(get(quantity)) + 40 - decrease --Center hydraulic system is more powerful than others
				else
					desired_pressure = (3000 + 100 * (pumps_on - 1)) * round(get(quantity)) - decrease
				end
				if get(pressure) < desired_pressure then
					if round(get(pressure)+increase*pumps_on) <= desired_pressure then
						set(pressure, get(pressure)+increase*pumps_on)
					else
						set(pressure, desired_pressure)
					end
				else
					if get(pressure) - 15 >= desired_pressure then
						set(pressure, get(pressure)-15)
					else
						set(pressure, desired_pressure)
					end
				end
			else
				local desired_pressure = 50 * round(get(quantity)) --default pressure is 50 psi unless more than half of the fluid leaks out
				if get(pressure) > desired_pressure then
					if i == 1 or i == 4 then
						if get(pressure)-95 >= desired_pressure then
							set(pressure, get(pressure)-95)
						else
							set(pressure, desired_pressure)
						end
					else
						if pumps_on == 0 or round(get(t_phi) / 180) % 2 == 1 then
							if get(pressure)-95 >= desired_pressure then
								set(pressure, get(pressure)-95)
							else
								set(pressure, desired_pressure)
							end
						end
					end
				end
			end
		end
		tmp = get(c_time)
	end
end

function UpdateQuantity()
	for i=1,4 do
		local p_temp = globalPropertyfae("Strato/777/hydraulics/pump/primary/temperatures", i)
		local d_temp = globalPropertyfae("Strato/777/hydraulics/pump/demand/temperatures", i)
		local sys_idx = GetSysIdx(i)
		local qty = globalPropertyfae("Strato/777/hydraulics/qty", sys_idx)
		local qty_initial = hyd_qty_initial[sys_idx]
		local increase = 0
		if math.abs(get(p_temp) - get(oat)) >= 20 then --the higher the temperature, the higher the oil density => qty will increase with temperature
			increase = math.floor((get(p_temp) - get(oat)) / 20) * 0.01
		elseif math.abs(get(d_temp) - get(oat)) >= 20 then
			increase = math.floor((get(d_temp) - get(oat)) / 20) * 0.01
		end
		if get(qty) ~= qty_initial + increase then
			set(qty, qty_initial + increase)
		end
	end
end

function update()
	UpdateSysLoad()
	UpdateQuantity()
	UpdateActualState()
	UpdateTemperatures(0.5)
	UpdatePressure(0.5)
	UpdateLights()
end

function onAirportLoaded() --set all the pressures and pumps to normal if engines are running
	local mixture = globalPropertyf("sim/cockpit2/engine/actuators/mixture_ratio_all")
	ADP_onground_past = get(on_ground)
	if get(mixture) == 1 then
		for i=1,4 do
			local demand = globalPropertyiae("Strato/777/hydraulics/pump/demand/state", i)
			local primary = globalPropertyiae("Strato/777/hydraulics/pump/primary/state", i)
			local sys_idx = GetSysIdx(i)
			local pressure = globalPropertyiae("Strato/777/hydraulics/press", sys_idx)
			set(pressure, 3000)
			set(demand, 1)
			set(primary, 1)
		end
	end
end

onAirportLoaded()

function onModuleDone()
	sasl.print("Shutting down")
end