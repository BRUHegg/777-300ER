addSearchPath(moduleDirectory .. "/Custom Module/AUTOFLT/FBW")

include("fbw_controllers.lua")
include("misc_tools.lua")


tgt_ias = globalPropertyf("sim/cockpit2/autopilot/airspeed_dial_kts")
autothr_arm = globalPropertyi("Strato/777/mcp/at_arm", 1)
spd_hold = globalPropertyi("Strato/777/mcp/spd_hold", 0)
toga = globalPropertyi("Strato/777/mcp/toga", 0)
throt_lim = createGlobalPropertyf("Strato/777/autothr/thr_lim", 0.95)
throt_res_rt = createGlobalPropertyf("Strato/777/autothr/throt_res_rt", 0.95)

thr_kp = createGlobalPropertyf("Strato/777/autothr_dbg/kp", 0.00004)
thr_ki = createGlobalPropertyf("Strato/777/autothr_dbg/ki", 0.000)
thr_kd = createGlobalPropertyf("Strato/777/autothr_dbg/kd", 0.00001)
thr_et = createGlobalPropertyf("Strato/777/autothr_dbg/et", 0)
thr_cmd = createGlobalPropertyf("Strato/777/autothr_dbg/cmd", 0)
thr_resp = createGlobalPropertyf("Strato/777/autothr_dbg/resp", 0.04)
pred_ias_kt = createGlobalPropertyf("Strato/777/autothr_dbg/pred_ias_kt", 0)
pred_ias_sec = createGlobalPropertyf("Strato/777/autothr_dbg/pred_ias_sec", 10)

throttle_cmd = globalPropertyf("sim/cockpit2/engine/actuators/throttle_ratio_all")
L_reverser_deployed = globalPropertyiae("sim/cockpit2/annunciators/reverser_on", 1)
R_reverser_deployed = globalPropertyiae("sim/cockpit2/annunciators/reverser_on", 2)

ra_pilot = globalPropertyf("sim/cockpit2/gauges/indicators/radio_altimeter_height_ft_pilot")
ra_copilot = globalPropertyf("sim/cockpit2/gauges/indicators/radio_altimeter_height_ft_copilot")
cas_pilot = globalPropertyf("sim/cockpit2/gauges/indicators/airspeed_kts_pilot")
cas_copilot = globalPropertyf("sim/cockpit2/gauges/indicators/airspeed_kts_copilot")

-- We use pitch here to limit idle thrust
pitch_pilot = globalPropertyf("sim/cockpit/gyros/the_ind_ahars_pilot_deg")
pitch_copilot = globalPropertyf("sim/cockpit/gyros/the_ind_ahars_copilot_deg")

ra_pilot = globalPropertyf("sim/cockpit2/gauges/indicators/radio_altimeter_height_ft_pilot")
ra_copilot = globalPropertyf("sim/cockpit2/gauges/indicators/radio_altimeter_height_ft_copilot")

vs_pilot_fpm = globalPropertyf("sim/cockpit2/gauges/indicators/vvi_fpm_pilot")
vs_copilot_fpm = globalPropertyf("sim/cockpit2/gauges/indicators/vvi_fpm_copilot")


f_time = globalPropertyf("sim/operation/misc/frame_rate_period")

AT_KP = 0.00004
AT_KP_EXTRA = 0.00008
AT_KI = 0.00001

autothrot_pid = PID:new{kp = AT_KP, ki = AT_KI, kd = 0, errtotal = 0, errlast = 0, lim_out = 1,  lim_et = 100}

ias_last = 0
gs_last = 0
ra_last = 0
pitch_last = 0
N_SEC_SPD_PREDICT = 10
PITCH_MAX = 15
PITCH_MIN = 0
THR_SERVO_RESPONSE = 0.002
AT_FLARE_ENTRY_FT = 100

at_flare_begin_ft = 0

AT_MODE_OFF = 0
AT_MODE_IAS_HOLD = 1
AT_MODE_RETARD = 2
AT_MODE_HOLD = 3
AT_MODE_THR_REF = 4

curr_at_mode = AT_MODE_OFF
at_engaged = false

function getThrottleIdleAltitudeFt(vs_entry)
    local curr_vs = lim(vs_entry, 0, -1500)
    local vs_calc = -1500 - curr_vs
    return 50 + (vs_calc / 1500) * 30
end

function setThrottleIASHoldCmd()
    at_engaged = true
    if get(f_time) ~= 0 then
        local avg_ias = (get(cas_pilot) + get(cas_copilot)) / 2
        local avg_pitch = (get(pitch_pilot) + get(pitch_copilot)) / 2
        avg_pitch = lim(avg_pitch, PITCH_MAX, PITCH_MIN)
        local min_idle = get(thr_resp) * avg_pitch

        local ias_accel = (avg_ias - ias_last) / get(f_time)
        local pitch_accel = (avg_pitch - pitch_last) / get(f_time)

        local ias_predict = avg_ias + ias_accel * N_SEC_SPD_PREDICT

        local tgt_kp = AT_KP + pitch_accel * 0.00001
        if math.abs(avg_ias-get(tgt_ias)) > 20 then
            tgt_kp = AT_KP_EXTRA  -- For the extra kick
        end
        set(pred_ias_kt, ias_predict)
        autothrot_pid:update{tgt=get(tgt_ias), curr=ias_predict}
        --set(thr_et, autothrot_pid.errtotal)
        set(thr_cmd, autothrot_pid.output)
        local autothr_cmd = lim(get(throttle_cmd)+autothrot_pid.output, get(throt_lim), min_idle)
        local thr_lvr_cmd = EvenChange(get(throttle_cmd), autothr_cmd, THR_SERVO_RESPONSE)
        set(throttle_cmd, thr_lvr_cmd)
        ias_last = avg_ias
        pitch_last = avg_pitch
    end
end

function setThrottleRetardCmd()
    local thr_lvr_cmd = EvenChange(get(throttle_cmd), 0, THR_SERVO_RESPONSE)
    set(throttle_cmd, thr_lvr_cmd)
end

function updateMode()
    local avg_ra = (get(ra_pilot) + get(ra_copilot)) / 2
    local avg_ias = (get(cas_pilot) + get(cas_copilot)) / 2

    if get(L_reverser_deployed) == 1 or get(R_reverser_deployed) == 1 then
        curr_at_mode = AT_MODE_OFF
        set(spd_hold, 0)
        ra_last = avg_ra
        return
    end
    if avg_ias <= 50 and get(autothr_arm) == 1 and get(toga) == 1 then
        curr_at_mode = AT_MODE_THR_REF
        at_engaged = false
    elseif avg_ias > 50 and curr_at_mode == AT_MODE_THR_REF then
        curr_at_mode = AT_MODE_HOLD
        at_engaged = false
    elseif get(autothr_arm) == 1 and get(spd_hold) == 1 then
        if (avg_ra > 400 and (curr_at_mode == AT_MODE_HOLD or 
            curr_at_mode == AT_MODE_OFF)) or at_engaged then 
            local vs_avg_fpm = (get(vs_pilot_fpm) + get(vs_copilot_fpm)) / 2
            if avg_ra < AT_FLARE_ENTRY_FT and ra_last > AT_FLARE_ENTRY_FT 
                and vs_avg_fpm < 0 then
                at_flare_begin_ft = getThrottleIdleAltitudeFt(vs_avg_fpm)
                curr_at_mode = AT_MODE_IAS_HOLD
            elseif avg_ra < at_flare_begin_ft and ra_last > at_flare_begin_ft then
                curr_at_mode = AT_MODE_RETARD
            elseif curr_at_mode ~= AT_MODE_RETARD then
                curr_at_mode = AT_MODE_IAS_HOLD
            end
        end
    elseif get(autothr_arm) ~= 1 then
        curr_at_mode = AT_MODE_OFF
    end
    ra_last = avg_ra
end


function update()
    updateMode()
    if curr_at_mode ~= AT_MODE_IAS_HOLD then
        ias_last = (get(cas_pilot) + get(cas_copilot)) / 2
        pitch_last = (get(pitch_pilot) + get(pitch_copilot)) / 2
    end
    if curr_at_mode == AT_MODE_IAS_HOLD then
        setThrottleIASHoldCmd()
    elseif curr_at_mode == AT_MODE_RETARD then
        setThrottleRetardCmd()
    end
end
