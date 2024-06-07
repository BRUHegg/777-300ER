pitch_kp = createGlobalPropertyf("Strato/777/pitch_dbg/kp", 0.0005)
pitch_ki = createGlobalPropertyf("Strato/777/pitch_dbg/ki", 0)
pitch_kd = createGlobalPropertyf("Strato/777/pitch_dbg/kd", 0)
pitch_et = createGlobalPropertyf("Strato/777/pitch_dbg/et", 0)
pitch_resp = createGlobalPropertyf("Strato/777/pitch_dbg/resp", 7)
vs_tgt = createGlobalPropertyf("Strato/777/pitch_dbg/vs_tgt", 0)
ap_pitch_eng = createGlobalPropertyi("Strato/777/pitch_dbg/eng", 0)
vs_pred_sec = createGlobalPropertyf("Strato/777/pitch_dbg/vs_pred_sec", 8)
vs_pred_fpm = createGlobalPropertyf("Strato/777/pitch_dbg/vs_pred_fpm", 0)

pitch_pilot = globalPropertyf("sim/cockpit/gyros/the_ind_ahars_pilot_deg")
pitch_copilot = globalPropertyf("sim/cockpit/gyros/the_ind_ahars_copilot_deg")

pitch_tgt = createGlobalPropertyf("Strato/777/autopilot/pitch_tgt_deg", 0)
ap_pitch_on = createGlobalPropertyi("Strato/777/autopilot/ap_pitch_on", 0)

--vs_tgt = globalPropertyf("sim/cockpit2/autopilot/vvi_dial/fpm")
ap_engaged = globalPropertyi("Strato/777/mcp/ap_on", 0)

-- Sim sensor datarefs
vs_pilot_fpm = globalPropertyf("sim/cockpit2/gauges/indicators/vvi_fpm_pilot")
vs_copilot_fpm = globalPropertyf("sim/cockpit2/gauges/indicators/vvi_fpm_copilot")


vshold_pid = PID:new{kp = 0, ki = 0, kd = 0, errtotal = 0, errlast = 0, lim_out = 1,  lim_et = 100}
-- >15000 kp=0.00001 kd=0.00016
-- <15000 kp=0.00003 kd=0.0003

ap_pitch_engaged = false
vshold_pitch_deg = 0

VSHOLD_PITCH_MAX_DEG = 15
VSHOLD_PITCH_MIN_DEG = -10

vs_last_ft = 0
flap_last = 0

function getAutopilotVSHoldCmd(pitch_cmd_prev, vs_cmd_fpm)
    if get(f_time) ~= 0 then
        local vs_avg_fpm = (get(vs_pilot_fpm) + get(vs_copilot_fpm)) / 2
        local vs_accel = (vs_avg_fpm - vs_last_ft) / get(f_time)
        local vs_pred = vs_avg_fpm + vs_accel * get(vs_pred_sec)
        set(vs_pred_fpm, vs_pred)

        local tgt_kp = 0.0005
        if round(get(pfc_flaps)) >= 15 then
            tgt_kp = 0.001
        end

        vshold_pid:update{kp=tgt_kp, ki=get(pitch_ki), kd=get(pitch_kd), tgt=vs_cmd_fpm, 
                curr=vs_pred}
        set(pitch_et, vshold_pid.errtotal)
        
        local vshold_cmd = lim(pitch_cmd_prev+vshold_pid.output * get(f_time), 
            VSHOLD_PITCH_MAX_DEG, VSHOLD_PITCH_MIN_DEG)
        --if get(pfc_flaps) > 6 and flap_last <= 6 then
        --    print(get(pfc_flaps), flap_last)
        --    vshold_cmd = vshold_cmd - get(pitch_resp)
        --elseif flap_last > 6 and get(pfc_flaps) <= 6 then
        --    vshold_cmd = vshold_cmd + get(pitch_resp)
        --    print(get(pfc_flaps), flap_last)
        --end
        set(pitch_tgt, vshold_cmd)
        vs_last_ft = vs_avg_fpm
        flap_last = get(pfc_flaps)
        return vshold_cmd
    end

    return 0
end

function getAutopilotPitchCmd()
    if get(ap_pitch_eng) == 1 then
        set(ap_pitch_on, 1)
        ap_pitch_engaged = true
        vshold_pitch_deg = getAutopilotVSHoldCmd(vshold_pitch_deg, get(vs_tgt))
    else
        set(ap_pitch_on, 0)
        vshold_pitch_deg = (get(pitch_pilot) + get(pitch_copilot)) / 2
        ap_pitch_engaged = false
        flap_last = get(pfc_flaps)
        vs_last_ft = (get(vs_pilot_fpm) + get(vs_copilot_fpm)) / 2
    end
    return vshold_pitch_deg
end
