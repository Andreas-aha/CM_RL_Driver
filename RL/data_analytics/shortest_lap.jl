using Glob
import JSON

println("hello world\n\n")

path = "/home/andreas-z97x-ud3h/CM_Projects/CM_RL_Driver/RL/rl_data/run20210101-no_sideslip_pen/rl_uaq_store"
files = glob([r"ep_[0-9]+_[0-9]+"], path)

lap_start = 10.
lap_end = 5483.
max_vel_start = 6.

rl_quants = Dict()
i = 2
fn = "/home/andreas-z97x-ud3h/CM_Projects/CM_RL_Driver/RL/rl_data/run20210101-no_sideslip_pen/rl_uaq_store/ep_12924_2.json"
println(fn)
open(fn, "r") do f
    global rl_quants
    rl_quants=JSON.parse(f)  # parse and transform data
end

sRoad = 0

for step_quants in rl_quants

    global sRoad

    prev_sRoad = sRoad
    sRoad = get(step_quants, "Vhcl.sRoad", -1)
    Vhcl_v = get(step_quants, "Vhcl.v", -1)
    sDiff = sRoad - prev_sRoad

    if abs(sDiff) > 20
        LapTime = get(step_quants, "Time", -1)
        println("newStart = $sRoad End = $prev_sRoad Time = $LapTime Vhcl.v = $Vhcl_v")
    end

end