addSearchPath(moduleDirectory .. "/Custom Module/AUTOFLT/FBW")

include("fbw_controllers.lua")
include("misc_tools.lua")


tgt_ias = createGlobalPropertyi("Strato/777/mcp/tgt_spd", 180)
autothr_arm = createGlobalPropertyi("Strato/777/mcp/at_arm", 1)
spd_hold = createGlobalPropertyi("Strato/777/mcp/spd_hold", 0)
throt_lim = createGlobalPropertyf("Strato/777/autothr/thr_lim", 0.95)
throt_res_rt = createGlobalPropertyf("Strato/777/autothr/throt_res_rt", 0.95)

thr_kp = createGlobalPropertyf("Strato/777/autothr_dbg/kp", 0.00006)
thr_ki = createGlobalPropertyf("Strato/777/autothr_dbg/ki", 0.00002)
thr_kd = createGlobalPropertyf("Strato/777/autothr_dbg/kd", 0.0009)
thr_et = createGlobalPropertyf("Strato/777/autothr_dbg/et", 0)
thr_cmd = createGlobalPropertyf("Strato/777/autothr_dbg/cmd", 0)
thr_resp = createGlobalPropertyf("Strato/777/autothr_dbg/resp", 0.002)

throttle_cmd = globalPropertyf("sim/cockpit2/engine/actuators/throttle_ratio_all")
ra_pilot = globalPropertyf("sim/cockpit2/gauges/indicators/radio_altimeter_height_ft_pilot")
ra_copilot = globalPropertyf("sim/cockpit2/gauges/indicators/radio_altimeter_height_ft_copilot")
cas_pilot = globalPropertyf("sim/cockpit2/gauges/indicators/airspeed_kts_pilot")
cas_copilot = globalPropertyf("sim/cockpit2/gauges/indicators/airspeed_kts_copilot")

autothrot_pid = PID:new{kp = 0, ki = 0, kd = 0, errtotal = 0, errlast = 0, lim_out = 1,  lim_et = 100}


function setThrottleCmd()
    local avg_spd = (get(cas_pilot) + get(cas_copilot)) / 2
    autothrot_pid:update{kp=get(thr_kp), ki=get(thr_ki), kd=get(thr_kd), tgt=get(tgt_ias), 
			curr=avg_spd}
    set(thr_et, autothrot_pid.errtotal)
    set(thr_cmd, autothrot_pid.output)
    local autothr_cmd = lim(get(throttle_cmd)+autothrot_pid.output, get(throt_lim), 0)
    --print(autothr_cmd-get(throttle_cmd))
    local thr_lvr_cmd = EvenChange(get(throttle_cmd), autothr_cmd, get(thr_resp))
    set(throttle_cmd, thr_lvr_cmd)
end


function update()
    if get(autothr_arm) == 1 and get(spd_hold) == 1 then
        setThrottleCmd()
    end
end
