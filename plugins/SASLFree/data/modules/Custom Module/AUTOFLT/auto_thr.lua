addSearchPath(moduleDirectory .. "/Custom Module/AUTOFLT/FBW")

include("fbw_controllers.lua")
include("misc_tools.lua")


tgt_ias = globalPropertyf("sim/cockpit2/autopilot/airspeed_dial_kts")
autothr_arm = createGlobalPropertyi("Strato/777/mcp/at_arm", 1)
spd_hold = createGlobalPropertyi("Strato/777/mcp/spd_hold", 0)
throt_lim = createGlobalPropertyf("Strato/777/autothr/thr_lim", 0.95)
throt_res_rt = createGlobalPropertyf("Strato/777/autothr/throt_res_rt", 0.95)

thr_kp = createGlobalPropertyf("Strato/777/autothr_dbg/kp", 0.00004)
thr_ki = createGlobalPropertyf("Strato/777/autothr_dbg/ki", 0.000)
thr_kd = createGlobalPropertyf("Strato/777/autothr_dbg/kd", 0.00001)
thr_et = createGlobalPropertyf("Strato/777/autothr_dbg/et", 0)
thr_cmd = createGlobalPropertyf("Strato/777/autothr_dbg/cmd", 0)
thr_resp = createGlobalPropertyf("Strato/777/autothr_dbg/resp", 0.002)
pred_ias_kt = createGlobalPropertyf("Strato/777/autothr_dbg/pred_ias_kt", 0)
pred_ias_sec = createGlobalPropertyf("Strato/777/autothr_dbg/pred_ias_sec", 10)

throttle_cmd = globalPropertyf("sim/cockpit2/engine/actuators/throttle_ratio_all")
ra_pilot = globalPropertyf("sim/cockpit2/gauges/indicators/radio_altimeter_height_ft_pilot")
ra_copilot = globalPropertyf("sim/cockpit2/gauges/indicators/radio_altimeter_height_ft_copilot")
cas_pilot = globalPropertyf("sim/cockpit2/gauges/indicators/airspeed_kts_pilot")
cas_copilot = globalPropertyf("sim/cockpit2/gauges/indicators/airspeed_kts_copilot")

f_time = globalPropertyf("sim/operation/misc/frame_rate_period")

autothrot_pid = PID:new{kp = 0.00004, ki = 0.00001, kd = 0, errtotal = 0, errlast = 0, lim_out = 1,  lim_et = 100}

speed_last = 0
N_SEC_SPD_PREDICT = 10

function setThrottleCmd()
    if get(f_time) ~= 0 then
        local avg_spd = (get(cas_pilot) + get(cas_copilot)) / 2
        local accel = (avg_spd - speed_last) / get(f_time)
        local spd_predict = avg_spd + accel * N_SEC_SPD_PREDICT
        set(pred_ias_kt, spd_predict)
        autothrot_pid:update{tgt=get(tgt_ias), curr=spd_predict}
        set(thr_et, autothrot_pid.errtotal)
        set(thr_cmd, autothrot_pid.output)
        local autothr_cmd = lim(get(throttle_cmd)+autothrot_pid.output, get(throt_lim), 0)
        local thr_lvr_cmd = EvenChange(get(throttle_cmd), autothr_cmd, 0.002)
        set(throttle_cmd, thr_lvr_cmd)
        speed_last = avg_spd
    end
end


function update()
    if get(autothr_arm) == 1 and get(spd_hold) == 1 then
        setThrottleCmd()
    else
        speed_last = (get(cas_pilot) + get(cas_copilot)) / 2
    end
end
