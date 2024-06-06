pitch_kp = createGlobalPropertyf("Strato/777/pitch_dbg/kp", 0.00001)
pitch_ki = createGlobalPropertyf("Strato/777/pitch_dbg/ki", 0)
pitch_kd = createGlobalPropertyf("Strato/777/pitch_dbg/kd", 0.0003)
pitch_et = createGlobalPropertyf("Strato/777/pitch_dbg/et", 0)
pitch_resp = createGlobalPropertyf("Strato/777/pitch_dbg/resp", 0)
vs_tgt = createGlobalPropertyf("Strato/777/pitch_dbg/vs_tgt", 0)
ap_pitch_eng = createGlobalPropertyi("Strato/777/pitch_dbg/eng", 0)

pitch_tgt = createGlobalPropertyf("Strato/777/autopilot/pitch_tgt_deg", 0)
ap_pitch_on = createGlobalPropertyi("Strato/777/autopilot/ap_pitch_on", 0)

--vs_tgt = globalPropertyf("sim/cockpit2/autopilot/vvi_dial/fpm")
ap_engaged = globalPropertyi("Strato/777/mcp/ap_on", 0)

-- Sim sensor datarefs
vs_pilot_fpm = globalPropertyf("sim/cockpit2/gauges/indicators/vvi_fpm_pilot")
vs_copilot_fpm = globalPropertyf("sim/cockpit2/gauges/indicators/vvi_fpm_copilot")


vshold_pid = PID:new{kp = 0, ki = 0, kd = 0, errtotal = 0, errlast = 0, lim_out = 1,  lim_et = 100}


ap_pitch_engaged = true
vshold_pitch_deg = 0

VSHOLD_PITCH_MAX_DEG = 15
VSHOLD_PITCH_MIN_DEG = -8

function getAutopilotVSHoldCmd(pitch_cmd_prev, vs_cmd_fpm)
    local cs_avg_fpm = (get(vs_pilot_fpm) + get(vs_copilot_fpm)) / 2
    vshold_pid:update{kp=get(pitch_kp), ki=get(pitch_ki), kd=get(pitch_kd), tgt=vs_cmd_fpm, 
			curr=cs_avg_fpm}
    set(pitch_et, vshold_pid.errtotal)
    
    local vshold_cmd = lim(pitch_cmd_prev+vshold_pid.output, 
        VSHOLD_PITCH_MAX_DEG, VSHOLD_PITCH_MIN_DEG)
    set(pitch_tgt, vshold_cmd)
    --local pitch_cmd_deg = EvenChange(pitch_cmd_prev, vshold_cmd, get(pitch_resp))
    return vshold_cmd
end

function getAutopilotPitchCmd()
    vshold_pitch_deg = getAutopilotVSHoldCmd(vshold_pitch_deg, get(vs_tgt))
    return vshold_pitch_deg
end
