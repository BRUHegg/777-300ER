pitch_kp = createGlobalPropertyf("Strato/777/pitch_dbg/kp", 0.0005)
pitch_ki = createGlobalPropertyf("Strato/777/pitch_dbg/ki", 0)
pitch_kd = createGlobalPropertyf("Strato/777/pitch_dbg/kd", 0)
pitch_et = createGlobalPropertyf("Strato/777/pitch_dbg/et", 0)
pitch_resp = createGlobalPropertyf("Strato/777/pitch_dbg/resp", -0.11)
vs_cmd = createGlobalPropertyf("Strato/777/pitch_dbg/vscmd", 0)
alt_cmd = createGlobalPropertyf("Strato/777/pitch_dbg/altcmd", 0)
vs_tgt = createGlobalPropertyf("Strato/777/pitch_dbg/vs_tgt", 0)
ap_pitch_eng = createGlobalPropertyi("Strato/777/pitch_dbg/eng", 0)
vshold_eng = createGlobalPropertyi("Strato/777/mcp/vshold", 0)
alt_hold_eng = createGlobalPropertyi("Strato/777/mcp/althold", 0)
curr_vert_mode = createGlobalPropertyi("Strato/777/fma/active_vert_mode", 0)

mcp_alt_val = globalPropertyi("sim/cockpit/autopilot/altitude")


vs_pred_sec = createGlobalPropertyf("Strato/777/pitch_dbg/vs_pred_sec", 8)
vs_pred_fpm = createGlobalPropertyf("Strato/777/pitch_dbg/vs_pred_fpm", 0)

pitch_pilot = globalPropertyf("sim/cockpit/gyros/the_ind_ahars_pilot_deg")
pitch_copilot = globalPropertyf("sim/cockpit/gyros/the_ind_ahars_copilot_deg")
-- Here we have a IAS acceleration feedback.
cas_pilot = globalPropertyf("sim/cockpit2/gauges/indicators/airspeed_kts_pilot")
cas_copilot = globalPropertyf("sim/cockpit2/gauges/indicators/airspeed_kts_copilot")

pitch_tgt = createGlobalPropertyf("Strato/777/autopilot/pitch_tgt_deg", 0)
ap_pitch_on = createGlobalPropertyi("Strato/777/autopilot/ap_pitch_on", 0)

--vs_tgt = globalPropertyf("sim/cockpit2/autopilot/vvi_dial/fpm")
ap_engaged = globalPropertyi("Strato/777/mcp/ap_on", 0)

-- Sim sensor datarefs
alt_pilot = globalPropertyf("sim/cockpit2/gauges/indicators/altitude_ft_pilot")
alt_copilot = globalPropertyf("sim/cockpit2/gauges/indicators/altitude_ft_copilot")
vs_pilot_fpm = globalPropertyf("sim/cockpit2/gauges/indicators/vvi_fpm_pilot")
vs_copilot_fpm = globalPropertyf("sim/cockpit2/gauges/indicators/vvi_fpm_copilot")

--Spoilers:
spoiler_L1_act = globalPropertyfae("sim/flightmodel2/wing/spoiler1_deg", 3) --#4
spoiler_L2_act = globalPropertyfae("sim/flightmodel2/wing/spoiler2_deg", 3)
spoiler_R1_act = globalPropertyfae("sim/flightmodel2/wing/spoiler1_deg", 4) --#11
spoiler_R2_act = globalPropertyfae("sim/flightmodel2/wing/spoiler2_deg", 4)
spoiler_6_act = globalPropertyfae("sim/flightmodel2/wing/speedbrake1_deg", 1)
spoiler_7_act = globalPropertyfae("sim/flightmodel2/wing/speedbrake2_deg", 1)
spoiler_8_act = globalPropertyfae("sim/flightmodel2/wing/speedbrake2_deg", 2)
spoiler_9_act = globalPropertyfae("sim/flightmodel2/wing/speedbrake1_deg", 2)


vshold_pid = PID:new{kp = 0, ki = 0, kd = 0, errtotal = 0, errlast = 0, lim_out = 1,  lim_et = 100}

ap_pitch_engaged = false
vshold_pitch_deg = 0

VSHOLD_PITCH_MAX_DEG = 20
VSHOLD_PITCH_MIN_DEG = -10

ALT_HOLD_ALT_REACH_SEC = 10
ALT_HOLD_DECEL = 10
ALT_HOLD_CAPTURE_ALT_FT = 150
ALT_HOLD_VS_CMD_MAX_FPM = 1000

vs_last_ft = 0
ias_last = 0

alt_hold_alt_tgt = 0
alt_hold_alt_blacklist = 0  -- This is to prevent re-engagement after forced disengage
vs_hold_vs_tgt = 0

alt_hold_alt_acq = false
alt_hold_mcp_alt_acq = false

VERT_MODE_OFF = 0
VERT_MODE_VSHOLD = 1
VERT_MODE_ALTHOLD = 2
VERT_MODE_FLC = 3

vert_mode = VERT_MODE_OFF

function getIASCorrection()
    if get(f_time) ~= 0 then
        local ias_avg_kts = (get(cas_pilot) + get(cas_pilot)) / 2
        local ias_accel = (ias_avg_kts - ias_last) / get(f_time)

        ias_last = ias_avg_kts

        return ias_accel * -0.11
    else
        return 0
    end
end

function getSpoilerCorrection()
    local sp_avg = (get(spoiler_L1_act) + get(spoiler_L2_act) + 
        get(spoiler_R1_act) + get(spoiler_R2_act) + get(spoiler_6_act) + 
        get(spoiler_7_act) + get(spoiler_8_act) + get(spoiler_9_act)) / 8
    return sp_avg * 0.07
end

function getAutopilotVSHoldCmd(pitch_cmd_prev, vs_cmd_fpm)
    if get(f_time) ~= 0 then
        local vs_avg_fpm = (get(vs_pilot_fpm) + get(vs_copilot_fpm)) / 2
        
        local vs_accel = (vs_avg_fpm - vs_last_ft) / get(f_time)

        local vs_pred = vs_avg_fpm + vs_accel * get(vs_pred_sec)
        set(vs_pred_fpm, vs_pred)

        local tgt_kp = 0.0005
        if round(get(pfc_flaps)) >= 5 then
            tgt_kp = 0.001
        end

        vshold_pid:update{kp=tgt_kp, ki=get(pitch_ki), kd=get(pitch_kd), tgt=vs_cmd_fpm, 
                curr=vs_pred}
        set(pitch_et, vshold_pid.errtotal)
        
        local tgt_cmd = pitch_cmd_prev+vshold_pid.output * get(f_time)

        local vshold_cmd = lim(tgt_cmd, VSHOLD_PITCH_MAX_DEG, VSHOLD_PITCH_MIN_DEG)
        set(pitch_tgt, vshold_cmd)
        vs_last_ft = vs_avg_fpm
        return vshold_cmd 
    end

    return 0
end

function getAutopilotVSHoldCmdFull()
    local alt_avg_ft = (get(alt_pilot) + get(alt_copilot)) / 2
    local vs_avg_fpm = (get(vs_pilot_fpm) + get(vs_copilot_fpm)) / 2

    local alt_err = get(mcp_alt_val) - alt_avg_ft
    local alt_intcpt_margin_ft = 500
    if vs_avg_fpm > 1300 then
        alt_intcpt_margin_ft = alt_intcpt_margin_ft + (vs_avg_fpm - 1300) * (450/1300)
    end
    if alt_err > 0 and alt_err <= alt_intcpt_margin_ft and vs_hold_vs_tgt > 0 then
        vs_hold_vs_tgt = lim(vs_hold_vs_tgt, 700, 0)
    elseif alt_err < 0 and alt_err >= -alt_intcpt_margin_ft and vs_hold_vs_tgt < 0 then
        vs_hold_vs_tgt = lim(vs_hold_vs_tgt, 0, -700)
    end
    vshold_pitch_deg = getAutopilotVSHoldCmd(vshold_pitch_deg, vs_hold_vs_tgt) 
    return vshold_pitch_deg + getIASCorrection() + getSpoilerCorrection()
end

function getAutopilotAltHoldCmd()
    local alt_avg_ft = (get(alt_pilot) + get(alt_copilot)) / 2
    local alt_err = alt_hold_alt_tgt - alt_avg_ft
    vs_hold_vs_tgt = alt_err * 2
    if math.abs(vs_hold_vs_tgt) > ALT_HOLD_VS_CMD_MAX_FPM then
        set(alt_hold_eng, 0)
        alt_hold_alt_blacklist = alt_hold_alt_tgt
        vert_mode = VERT_MODE_VSHOLD
        vs_hold_vs_tgt = get(vs_tgt)
    else
        alt_hold_alt_blacklist = 0
    end
    return getAutopilotVSHoldCmdFull()
end

function updateMode()
    local alt_avg_ft = (get(alt_pilot) + get(alt_copilot)) / 2
    if math.abs(get(mcp_alt_val) - alt_avg_ft) > ALT_HOLD_CAPTURE_ALT_FT and
        get(alt_hold_eng) == 1 and get(vshold_eng) == 1 and vert_mode == VERT_MODE_ALTHOLD then
        set(alt_hold_eng, 0)
        alt_hold_alt_acq = false
    end
    if get(alt_hold_eng) == 1 and get(mcp_alt_val) ~= alt_hold_alt_blacklist then
        local vs_avg_fpm = (get(vs_pilot_fpm) + get(vs_copilot_fpm)) / 2
        if math.abs(get(mcp_alt_val) - alt_avg_ft) > ALT_HOLD_CAPTURE_ALT_FT and 
            vert_mode ~= VERT_MODE_ALTHOLD then
            vs_hold_vs_tgt = 0
            vert_mode = VERT_MODE_VSHOLD
            alt_hold_alt_acq = true
        elseif math.abs(get(mcp_alt_val) - alt_avg_ft) <= ALT_HOLD_CAPTURE_ALT_FT and 
            vert_mode ~= VERT_MODE_ALTHOLD then
            vert_mode = VERT_MODE_ALTHOLD
            set(vs_tgt, 0)
            alt_hold_alt_tgt = get(mcp_alt_val)
        end
        if alt_hold_alt_acq and vs_avg_fpm < 100 then
            alt_hold_alt_acq = false
            vert_mode = VERT_MODE_ALTHOLD
            alt_hold_alt_tgt = alt_avg_ft
            set(vs_tgt, 0)
        end
    elseif get(vshold_eng) == 1 then
        if math.abs(get(mcp_alt_val) - alt_avg_ft) <= ALT_HOLD_CAPTURE_ALT_FT then
            set(alt_hold_eng, 1)
            set(vshold_eng, 0)
        else
            vs_hold_vs_tgt = get(vs_tgt)
        end
        vert_mode = VERT_MODE_VSHOLD
    else
        vert_mode = VERT_MODE_OFF
    end
    if vert_mode ~= VERT_MODE_OFF then
        set(ap_pitch_on, 1)
    else
        set(ap_pitch_on, 0)
    end
    set(alt_cmd, alt_hold_alt_tgt)
    set(vs_cmd, vs_hold_vs_tgt)
end

function getAutopilotPitchCmd()
    updateMode()
    local vshold_cmd_deg = 0
    if vert_mode ~= VERT_MODE_OFF then
        set(ap_pitch_on, 1)
        ap_pitch_engaged = true
        if vert_mode == VERT_MODE_VSHOLD then
            vshold_cmd_deg = getAutopilotVSHoldCmdFull()
        else
            vshold_cmd_deg = getAutopilotAltHoldCmd()
        end
    else
        set(ap_pitch_on, 0)
        vshold_pitch_deg = (get(pitch_pilot) + get(pitch_copilot)) / 2
        vshold_cmd_deg = vshold_pitch_deg
        ap_pitch_engaged = false
        vs_last_ft = (get(vs_pilot_fpm) + get(vs_copilot_fpm)) / 2
        ias_last = (get(cas_pilot) + get(cas_pilot)) / 2
    end
    set(curr_vert_mode, vert_mode)
    return vshold_cmd_deg
end
