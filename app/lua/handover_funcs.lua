----------------------------------
-- functions MOTOR - HANDOVER        --
----------------------------------

function HANDOVER_prepare(port)
    local wb = yarp.Bottle()
    local reply = yarp.Bottle()
    wb:clear()
    wb:addString("arm")
    port:write(wb,reply)
    return reply:get(0):asString()
end

function HANDOVER_acquire(port)
    local wb = yarp.Bottle()
    local reply = yarp.Bottle()
    wb:clear()
    wb:addString("go_acquire")
    port:write(wb,reply)
    yarp.Time_delay(1.5)
    return reply:get(0):asString()
end

function HANDOVER_filter(port)
   local wb = yarp.Bottle()
   local reply = yarp.Bottle()
   wb:clear()
    wb:addString("go_filter")
    port:write(wb,reply)
    yarp.Time_delay(1.5)
    return reply:get(0):asString()
end

function HANDOVER_try_again(port)
   local wb = yarp.Bottle()
   local reply = yarp.Bottle()
   wb:clear()
    wb:addString("try_again")
    wb:addInt(0)
    port:write(wb,reply)
    yarp.Time_delay(4.0)
    return reply:get(0):asString()
end

function HANDOVER_localize(port)
   local wb = yarp.Bottle()
   local reply = yarp.Bottle()
   wb:clear()
    yarp.Time_delay(1.5)
    wb:addString("go_localize")
    port:write(wb,reply)
    yarp.Time_delay(50.0)
    return reply:get(0):asString()
end

function HANDOVER_ask_pose(port)
   local wb = yarp.Bottle()
   local reply = yarp.Bottle()
   wb:clear()
    wb:addString("ask_new_pose")
    port:write(wb,reply)
    yarp.Time_delay(3.0)
    return reply:get(0):asString()
end

function HANDOVER_move_first_hand(port)
   local wb = yarp.Bottle()
   local reply = yarp.Bottle()
   wb:clear()
    wb:addString("move")
    wb:addString("first_hand")
    port:write(wb,reply)
    yarp.Time_delay(5.0)
    return reply:get(0):asString()
end

function HANDOVER_set_waypoint(port)
   local wb = yarp.Bottle()
   local reply = yarp.Bottle()
   wb:clear()
    wb:addString("set_waypoint")
    wb:addInt(0)
    port:write(wb,reply)
    yarp.Time_delay(5.0)
    return reply:get(0):asString()
end

function HANDOVER_reach_final(port)
   local wb = yarp.Bottle()
   local reply = yarp.Bottle()
   wb:clear()
    wb:addString("reach_final")
    port:write(wb,reply)
    yarp.Time_delay(5.0)
    return reply:get(0):asString()
end

function HANDOVER_go_home(port)
   local wb = yarp.Bottle()
   local reply = yarp.Bottle()
   wb:clear()
    wb:addString("go_home")
    port:write(wb,reply)
    yarp.Time_delay(4.0)
    return reply:get(0):asString()
end

function HANDOVER_open_hand(port)
   local wb = yarp.Bottle()
   local reply = yarp.Bottle()
   wb:clear()
    wb:addString("open")
    port:write(wb)
    -- return reply:get(0):asString()
end

function HANDOVER_close_hand(port)
   local wb = yarp.Bottle()
   local reply = yarp.Bottle()
   wb:clear()
    wb:addString("grasp")
    port:write(wb)
    yarp.Time_delay(10.0)
    -- return reply:get(0):asString()
end

function HANDOVER_look_in_front(port)
   local wb = yarp.Bottle()
   local reply = yarp.Bottle()
   wb:clear()
    wb:addString("look_in_front")
    port:write(wb,reply)
    return reply:get(0):asString()
end

function HANDOVER_go_on(port)
   local reply = yarp.Bottle()
    port:read(reply)
    return reply:get(0):asString()
end
