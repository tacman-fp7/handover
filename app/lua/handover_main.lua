#!/usr/bin/lua

require("yarp")
require("rfsm")

yarp.Network()

shouldExit = false

-- initialization
in_hand_seg_port = yarp.Port()
in_hand_loc_port = yarp.Port()
pose_sel_port    = yarp.Port()
go_on_port  = yarp.Port()

-- load state machine model and initalize it

rf = yarp.ResourceFinder()
rf:setDefaultContext("handover/lua")
rf:configure(arg)
fsm_file = rf:findFile("handover_root_fsm.lua")
fsm_model = rfsm.load(fsm_file)
fsm = rfsm.init(fsm_model)
rfsm.run(fsm)

repeat
    rfsm.run(fsm)
    yarp.Time_delay(0.1)
until shouldExit ~= false

print("finishing")
-- Deinitialize yarp network
yarp.Network_fini()
