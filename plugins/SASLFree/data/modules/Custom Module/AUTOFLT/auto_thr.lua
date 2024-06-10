addSearchPath(moduleDirectory .. "/Custom Module/AUTOFLT/FBW")

include("fbw_controllers.lua")
include("misc_tools.lua")


tgt_ias = globalPropertyf("sim/cockpit2/autopilot/airspeed_dial_kts")
autothr_arm = globalPropertyi("Strato/777/mcp/at_arm", 1)
spd_hold = globalPropertyi("Strato/777/mcp/spd_hold", 0)
toga = globalPropertyi("Strato/777/mcp/toga", 0)
at_disc = globalPropertyi("Strato/777/mcp/at_disc", 0)
n1_lim = createGlobalPropertyf("Strato/777/autothr/n1_lim", 95)
throt_res_rt = createGlobalPropertyf("Strato/777/autothr/throt_res_rt", 0.95)

thr_kp = createGlobalPropertyf("Strato/777/autothr_dbg/kp", 0.000045)
thr_ki = createGlobalPropertyf("Strato/777/autothr_dbg/ki", 0.000)
thr_kd = createGlobalPropertyf("Strato/777/autothr_dbg/kd", 0.00058)
thr_et = createGlobalPropertyf("Strato/777/autothr_dbg/et", 0)
thr_cmd = createGlobalPropertyf("Strato/777/autothr_dbg/cmd", 0)
thr_resp = createGlobalPropertyf("Strato/777/autothr_dbg/resp", 0.04)
pred_ias_kt = createGlobalPropertyf("Strato/777/autothr_dbg/pred_ias_kt", 0)
pred_ias_sec = createGlobalPropertyf("Strato/777/autothr_dbg/pred_ias_sec", 10)

autothr_mode_dr = createGlobalPropertyi("Strato/777/fma/at_mode", 0)
curr_vert_mode = globalPropertyi("Strato/777/fma/active_vert_mode")
alt_acq = globalPropertyi("Strato/777/fma/alt_acq")

throttle_cmd = globalPropertyf("sim/cockpit2/engine/actuators/throttle_ratio_all")
L_reverser_deployed = globalPropertyiae("sim/cockpit2/annunciators/reverser_on", 1)
R_reverser_deployed = globalPropertyiae("sim/cockpit2/annunciators/reverser_on", 2)

ra_pilot = globalPropertyf("sim/cockpit2/gauges/indicators/radio_altimeter_height_ft_pilot")
ra_copilot = globalPropertyf("sim/cockpit2/gauges/indicators/radio_altimeter_height_ft_copilot")
cas_pilot = globalPropertyf("sim/cockpit2/gauges/indicators/airspeed_kts_pilot")
cas_copilot = globalPropertyf("sim/cockpit2/gauges/indicators/airspeed_kts_copilot")
gs_dref = globalPropertyf("sim/cockpit2/gauges/indicators/ground_speed_kt")

-- We use pitch here to limit idle thrust
pitch_pilot = globalPropertyf("sim/cockpit/gyros/the_ind_ahars_pilot_deg")
pitch_copilot = globalPropertyf("sim/cockpit/gyros/the_ind_ahars_copilot_deg")

ra_pilot = globalPropertyf("sim/cockpit2/gauges/indicators/radio_altimeter_height_ft_pilot")
ra_copilot = globalPropertyf("sim/cockpit2/gauges/indicators/radio_altimeter_height_ft_copilot")

vs_pilot_fpm = globalPropertyf("sim/cockpit2/gauges/indicators/vvi_fpm_pilot")
vs_copilot_fpm = globalPropertyf("sim/cockpit2/gauges/indicators/vvi_fpm_copilot")

engn_L_n1 = globalPropertyfae("sim/flightmodel/engine/ENGN_N1_", 1)
engn_R_n1 = globalPropertyfae("sim/flightmodel/engine/ENGN_N1_", 2)

f_time = globalPropertyf("sim/operation/misc/frame_rate_period")

AT_KP = 0.000045
AT_KD = 0.00058

ias_last = 0
gs_last = 0
ra_last = 0
PITCH_MAX = 15
PITCH_MIN = 0
THR_SERVO_RESPONSE = 0.002
AT_FLARE_ENTRY_FT = 100
THR_MAX_N1_DEV = 4

at_flare_begin_ft = 0
spd_flc_start_kts = 0

AT_MODE_OFF = 0
AT_MODE_IAS_HOLD = 1
AT_MODE_RETARD = 2
AT_MODE_HOLD = 3
AT_MODE_THR_REF = 4
AT_MODE_FLC_RETARD = 5
AT_MODE_FLC_REF = 6

curr_at_mode = AT_MODE_OFF
at_engaged = false

function getThrottleIdleAltitudeFt(vs_entry)
    local curr_vs = lim(vs_entry, 0, -1500)
    local vs_calc = -1500 - curr_vs
    return 50 + (vs_calc / 1500) * 30
end

function getThrottleN1HoldCmd()  -- Just holds the n1 at n1_lim
    local avg_n1 = (get(engn_L_n1) + get(engn_R_n1)) / 2
    local n1_err = get(n1_lim) - avg_n1
    return AT_KP * n1_err
end

function setThrottleIASHoldCmd(ias_tgt_kts)
    at_engaged = true
    if get(f_time) ~= 0 then
        local avg_ias = (get(cas_pilot) + get(cas_copilot)) / 2
        local avg_pitch = (get(pitch_pilot) + get(pitch_copilot)) / 2
        local avg_vs = (get(vs_pilot_fpm) + get(vs_copilot_fpm)) / 2
        local avg_n1 = (get(engn_L_n1) + get(engn_R_n1)) / 2
        local curr_gs = get(gs_dref)

        avg_pitch = lim(avg_pitch, PITCH_MAX, PITCH_MIN)
        local min_idle = get(thr_resp) * avg_pitch

        local ias_accel = (avg_ias - ias_last) / get(f_time)
        local gs_accel = (curr_gs - gs_last) / get(f_time)

        local ias_err = ias_tgt_kts - avg_ias
        local at_out = AT_KP * ias_err - AT_KD * (gs_accel - (avg_vs / 16000))
        set(thr_cmd, at_out)
        local autothr_cmd = lim(get(throttle_cmd)+at_out, 1, min_idle)
        if autothr_cmd > get(throttle_cmd) and 
            (math.abs(avg_n1 - get(n1_lim)) <= THR_MAX_N1_DEV or avg_n1 > get(n1_lim)) then
            local n1_hold_cmd = getThrottleN1HoldCmd()
            autothr_cmd = lim(get(throttle_cmd)+n1_hold_cmd, 1, min_idle)
        end
        local thr_lvr_cmd = EvenChange(get(throttle_cmd), autothr_cmd, THR_SERVO_RESPONSE)
        set(throttle_cmd, thr_lvr_cmd)
        ias_last = avg_ias
        gs_last = curr_gs
    end
end

function setThrottleRetardCmd()
    local thr_lvr_cmd = EvenChange(get(throttle_cmd), 0, THR_SERVO_RESPONSE)
    set(throttle_cmd, thr_lvr_cmd)
end

function setThrottleRefCmd()
    local n1_hold_cmd = getThrottleN1HoldCmd()
    local autothr_cmd = lim(get(throttle_cmd)+n1_hold_cmd, 1, 0)
    local thr_lvr_cmd = EvenChange(get(throttle_cmd), autothr_cmd, THR_SERVO_RESPONSE)
    set(throttle_cmd, thr_lvr_cmd)
end

function setThrottleFlcCmd(v_mode)
    if v_mode == 3 then  -- Flc climb
        local tgt_spd_kts = math.max(get(tgt_ias)+35, spd_flc_start_kts+35)
        setThrottleIASHoldCmd(tgt_spd_kts)
    else
        local tgt_spd_kts = math.min(get(tgt_ias)-8, spd_flc_start_kts-8)
        setThrottleIASHoldCmd(tgt_spd_kts)
    end
end

function updateMode(v_mode)
    local avg_ra = (get(ra_pilot) + get(ra_copilot)) / 2
    local avg_ias = (get(cas_pilot) + get(cas_copilot)) / 2

    if get(L_reverser_deployed) == 1 or get(R_reverser_deployed) == 1 or 
        get(at_disc) == 1 then
        curr_at_mode = AT_MODE_OFF
        set(spd_hold, 0)
        set(toga, 0)
        ra_last = avg_ra
        return
    end
    if (v_mode < 3 or get(alt_acq) == 1) and curr_at_mode >= AT_MODE_FLC_RETARD then
        set(spd_hold, 1)
    end
    if avg_ias <= 50 and get(autothr_arm) == 1 and get(toga) == 1 and 
        curr_at_mode ~= AT_MODE_HOLD then
        curr_at_mode = AT_MODE_THR_REF
        at_engaged = false
    elseif avg_ias > 50 and curr_at_mode == AT_MODE_THR_REF and v_mode == 0 then
        curr_at_mode = AT_MODE_HOLD
        at_engaged = false
    elseif (avg_ra > 400 or at_engaged) and v_mode >= 3 and get(alt_acq) == 0 then
        if curr_at_mode < AT_MODE_FLC_REF then
            spd_flc_start_kts = ias_last
        end
        if v_mode == 3 then
            curr_at_mode = AT_MODE_FLC_REF
        else
            curr_at_mode = AT_MODE_FLC_RETARD
        end
        set(spd_hold, 0)
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
    local v_mode = get(curr_vert_mode)
    updateMode(v_mode)
    if curr_at_mode ~= AT_MODE_IAS_HOLD then
        ias_last = (get(cas_pilot) + get(cas_copilot)) / 2
        gs_last = get(gs_dref)
    end
    if curr_at_mode == AT_MODE_IAS_HOLD then
        setThrottleIASHoldCmd(get(tgt_ias))
    elseif curr_at_mode == AT_MODE_RETARD then
        setThrottleRetardCmd()
    elseif curr_at_mode == AT_MODE_THR_REF then
        setThrottleRefCmd()
    elseif curr_at_mode >= AT_MODE_FLC_RETARD then
        setThrottleFlcCmd(v_mode)
    end
    set(autothr_mode_dr, curr_at_mode)
end
