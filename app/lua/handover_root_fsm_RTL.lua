#!/usr/bin/lua

dofile(rf:findFile("handover_funcs.lua"))

return rfsm.state {

   ----------------------------------
   -- state INITPORTS                  --
   ----------------------------------
   ST_INITPORTS = rfsm.state{
           entry=function()
                   ret = in_hand_seg_port:open("/handover/seg")
                   ret = ret and in_hand_loc_port:open("/handover/loc")
                   ret = ret and pose_sel_port:open("/handover/pos")
                   ret = ret and clos_chain_port:open("/handover/clos")
                   ret = ret and stable_grasp_l_port:open("/handover/grasp:l")
                   ret = ret and stable_grasp_r_port:open("/handover/grasp:r")
                   ret = ret and go_on_port:open("/handover/go_on")
                   if ret == false then
                           rfsm.send_events(fsm, 'e_error')
                   else
                           rfsm.send_events(fsm, 'e_connect')
                   end
           end
},

----------------------------------
   -- state CONNECTPORTS           --
   ----------------------------------
   ST_CONNECTPORTS = rfsm.state{
           entry=function()
                   ret = yarp.NetworkBase_connect(in_hand_seg_port:getName(), "/in-hand-segmentation/rpc")
                   ret =  ret and yarp.NetworkBase_connect(in_hand_loc_port:getName(), "/in-hand-localizer/rpc")
                   ret =  ret and yarp.NetworkBase_connect(pose_sel_port:getName(), "/pose-selection/rpc")
                   ret =  ret and yarp.NetworkBase_connect(clos_chain_port:getName(), "/closed-chain/rpc")
                   ret =  ret and yarp.NetworkBase_connect(stable_grasp_l_port:getName(), "/stableGrasp/left_hand/cmd:i")
                   ret =  ret and yarp.NetworkBase_connect(stable_grasp_r_port:getName(), "/stableGrasp/right_hand/cmd:i")
                   ret =  ret and yarp.NetworkBase_connect("/port/go_on", go_on_port:getName())
                   if ret == false then
                           print("\n\nERROR WITH CONNECTIONS, PLEASE CHECK\n\n")
                           rfsm.send_events(fsm, 'e_error')
                   end
           end
},


----------------------------------
  -- state FATAL                  --
  ----------------------------------
  ST_FATAL = rfsm.state{
          entry=function()
                  print("Fatal!")
                  shouldExit = true;
          end
},



----------------------------------
   -- state FINI                   --
   ----------------------------------
   ST_FINI = rfsm.state{
           entry=function()
                   print("Closing...")
                   yarp.NetworkBase_disconnect(in_hand_seg_port:getName(), "/in-hand-segmentation/rpc")
                   yarp.NetworkBase_disconnect(in_hand_loc_port:getName(), "/in-hand-localizer/rpc")
                   yarp.NetworkBase_disconnect(pose_sel_port:getName(), "/pose-selection/rpc")
                   yarp.NetworkBase_disconnect(clos_chain_port:getName(), "/closed-chain/rpc")
                   yarp.NetworkBase_disconnect(stable_grasp_l_port:getName(), "/stableGrasp/left_hand/cmd:i")
                   yarp.NetworkBase_disconnect(stable_grasp_r_port:getName(), "/stableGrasp/right_hand/cmd:i")
                   in_hand_seg_port:close()
                   in_hand_loc_port:close()
                   pose_sel_port:close()
                   clos_chain_port:close()

                   shouldExit = true;
           end
},


----------------------------------
  -- state PREPARE_SECOND_HAND                 --
  ----------------------------------
  ST_PREPARE_SECOND_HAND = rfsm.state{
          entry=function()
                  print(" preparing hands ..")
                  HANDOVER_open_hand(stable_grasp_l_port)
          end
},


----------------------------------
  -- state PC_ACQUISITION                   --
  ----------------------------------
  ST_PC_ACQ = rfsm.state{
          entry=function()
                  print("acquiring point cloud")
                  local ret = HANDOVER_acquire(in_hand_seg_port)
                  if ret == "fail" then
                      rfsm.send_events(fsm, 'e_error')
                  end
          end
},

----------------------------------
  -- state PC_FILT                 --
  ----------------------------------
  ST_PC_FILT = rfsm.state{
          entry=function()
                  print(" filtering points ...")
                  local ret = HANDOVER_filter(in_hand_seg_port)

                  if ret == "fail" then
                      rfsm.send_events(fsm, 'e_error')
                  end
          end
},

----------------------------------
  -- state LOC_POINTS_ACQ                   --
  ----------------------------------
  ST_LOC_POINTS_ACQ = rfsm.state{
          entry=function()
                  print("acquiring points for localization")
                  local ret = HANDOVER_acquire(in_hand_loc_port)

                  if ret == "fail" then
                      rfsm.send_events(fsm, 'e_error')
                  end
          end
},

----------------------------------
  -- state LOC_POINTS                  --
  ----------------------------------
  ST_LOC_POINTS = rfsm.state{
          entry=function()
                  print(" localizing object...")
                  local ret = HANDOVER_localize(in_hand_loc_port)

                  if ret == "fail" then
                      rfsm.send_events(fsm, 'e_error')
                  end
          end
},

----------------------------------
  -- state ASK_POSE                 --
  ----------------------------------
  ST_ASK_POSE = rfsm.state{
          entry=function()
                  print(" asking pose...")
                  local ret = HANDOVER_ask_pose(pose_sel_port)

                  if ret == "fail" then
                      rfsm.send_events(fsm, 'e_error')
                  end
          end
},

----------------------------------
  -- state MOVE_FIRST_HAND                --
  ----------------------------------
  ST_MOVE_FIRST_HAND = rfsm.state{
          entry=function()
                  print(" moving first hand...")
                  local ret = HANDOVER_move_first_hand(clos_chain_port)

                  if ret == "fail" then
                      rfsm.send_events(fsm, 'e_error')
                  end
          end
},

----------------------------------
  -- state SET_WAYPOINT                --
  ----------------------------------
  ST_SET_WAYPOINT = rfsm.state{
          entry=function()
                  print(" moving second hand to waypoint...")
                  local ret = HANDOVER_set_waypoint(pose_sel_port)

                  if ret == "fail" then
                      rfsm.send_events(fsm, 'e_error')
                  end
          end
},

----------------------------------
  -- state REACH_FINAL                --
  ----------------------------------
  ST_REACH_FINAL = rfsm.state{
          entry=function()
                  print(" moving second hand to final pose...")
                  local ret = HANDOVER_reach_final(pose_sel_port)

                  if ret == "fail" then
                      rfsm.send_events(fsm, 'e_error')
                  end
          end
},

----------------------------------
  -- state SET_WAYPOINT_BACK                --
  ----------------------------------
  ST_SET_WAYPOINT_BACK = rfsm.state{
          entry=function()
                  print(" moving second hand to waypoint...")
                  local ret = HANDOVER_set_waypoint(pose_sel_port)

                  if ret == "fail" then
                      rfsm.send_events(fsm, 'e_error')
                  end
          end
},

----------------------------------
  -- state CLOSE_SECOND_HAND                  --
  ----------------------------------
  ST_CLOSE_HAND = rfsm.state{
          entry=function()
                  print(" closing left hand ..")
                  HANDOVER_close_hand(stable_grasp_l_port)
          end
},

----------------------------------
  -- state CLOSE_FIRST_HAND                  --
  ----------------------------------
  ST_CLOSE_FIRST_HAND = rfsm.state{
          entry=function()
                  print(" closing right hand ..")
                  HANDOVER_close_hand(stable_grasp_r_port)
          end
},


----------------------------------
  -- state OPEN_FIRST_HAND                --
  ----------------------------------
  ST_OPEN_FIRST_HAND = rfsm.state{
          entry=function()
                  print(" opening right hand ..")
                  HANDOVER_open_hand(stable_grasp_r_port)
          end
},

----------------------------------
  -- state LOOK_IN_FRONT               --
  ----------------------------------
  ST_LOOK_IN_FRONT = rfsm.state{
          entry=function()
                  print(" looking in front ...")
                  local ret = HANDOVER_look_in_front(in_hand_seg_port)

                  if ret == "fail" then
                      rfsm.send_events(fsm, 'e_error')
                  end
          end
},

----------------------------------
  -- state GO_HOME                --
  ----------------------------------
  ST_GO_HOME = rfsm.state{
          entry=function()
                  print(" going home ...")
                  local ret = HANDOVER_go_home(clos_chain_port)

                  if ret == "fail" then
                      rfsm.send_events(fsm, 'e_error')
                  end
          end
},

----------------------------------
  -- state GO_ON                --
  ----------------------------------
  ST_GO_ON = rfsm.state{
          entry=function()
                  print(" go on or not?")
                  local ret = HANDOVER_go_on(go_on_port)
                  if ret == "go_on" then
                      rfsm.send_events(fsm, 'e_done')
                  end

                  if ret == "stop" then
                      rfsm.send_events(fsm, 'e_stop')
                  end

                  if ret == "again" then
                      rfsm.send_events(fsm, 'e_again')
                  end

                  if ret == "fail" then
                      rfsm.send_events(fsm, 'e_error')
                  end
          end
},

----------------------------------
  -- state GO_ON2                --
  ----------------------------------
  ST_GO_ON_2 = rfsm.state{
          entry=function()
                  print(" go on or back?")
                  local ret = HANDOVER_go_on(go_on_port)
                  if ret == "go_on" then
                      rfsm.send_events(fsm, 'e_go')
                  elseif ret == "stop" then
                      rfsm.send_events(fsm, 'e_stop')
	          elseif ret == "again" then
                      rfsm.send_events(fsm, 'e_again')
                  elseif ret == "fail" then
                      rfsm.send_events(fsm, 'e_error')
                  end
          end
},

----------------------------------
  -- state TRY_AGAIN               --
  ----------------------------------
  ST_TRY_AGAIN = rfsm.state{
          entry=function()
                  print("trying again ...")
                  local ret = HANDOVER_try_again(in_hand_seg_port)

                  if ret == "fail" then
                      rfsm.send_events(fsm, 'e_error')
                  end
          end
},

----------------------------------
  -- state AGAIN               --
  ----------------------------------
  ST_AGAIN = rfsm.state{
          entry=function()
                  print("again ...")
                  
          end
},





ST_INTERACT = interact_fsm,


 rfsm.transition { src='initial', tgt='ST_INITPORTS' },
 rfsm.transition { src='ST_INITPORTS', tgt='ST_CONNECTPORTS', events={ 'e_connect' } },
 rfsm.transition { src='ST_INITPORTS', tgt='ST_FATAL', events={ 'e_error' } },
 rfsm.transition { src='ST_CONNECTPORTS', tgt='ST_FINI', events={ 'e_error' } },
 rfsm.transition { src='ST_CONNECTPORTS', tgt='ST_TRY_AGAIN', events={ 'e_done' } },
 rfsm.transition { src='ST_TRY_AGAIN', tgt='ST_TRY_AGAIN', events={ 'e_error' } },
 rfsm.transition { src='ST_TRY_AGAIN', tgt='ST_CLOSE_FIRST_HAND', events={ 'e_done' } },
 rfsm.transition { src='ST_TRY_AGAIN', tgt='ST_PREPARE_SECOND_HAND', events={ 'e_done' } },
 rfsm.transition { src='ST_CLOSE_FIRST_HAND', tgt='ST_PC_ACQ', events={ 'e_done' } },
 rfsm.transition { src='ST_AGAIN', tgt='ST_PC_ACQ', events={ 'e_done' } },
 rfsm.transition { src='ST_PC_ACQ', tgt='ST_PC_ACQ', events={ 'e_error' } },
 rfsm.transition { src='ST_PC_ACQ', tgt='ST_PC_FILT', events={ 'e_done' } },
 rfsm.transition { src='ST_PC_FILT', tgt='ST_PC_FILT', events={ 'e_error' } },
 rfsm.transition { src='ST_PC_FILT', tgt='ST_LOC_POINTS_ACQ', events={ 'e_done' } },
 rfsm.transition { src='ST_LOC_POINTS_ACQ', tgt='ST_LOC_POINTS_ACQ', events={ 'e_error' } },
 rfsm.transition { src='ST_LOC_POINTS_ACQ', tgt='ST_LOC_POINTS', events={ 'e_done' } },
 rfsm.transition { src='ST_LOC_POINTS', tgt='ST_ASK_POSE', events={ 'e_done' } },
 rfsm.transition { src='ST_ASK_POSE', tgt='ST_ASK_POSE', events={ 'e_error' } },
 rfsm.transition { src='ST_ASK_POSE', tgt='ST_GO_ON', events={ 'e_done' } },
 rfsm.transition { src='ST_GO_ON', tgt='ST_GO_ON', events={ 'e_error' } },
 rfsm.transition { src='ST_GO_ON', tgt='ST_FATAL', events={ 'e_stop' } },
 rfsm.transition { src='ST_GO_ON', tgt='ST_AGAIN', events={ 'e_again' } },
 rfsm.transition { src='ST_GO_ON', tgt='ST_MOVE_FIRST_HAND', events={ 'e_done' } },
 rfsm.transition { src='ST_MOVE_FIRST_HAND', tgt='ST_SET_WAYPOINT', events={ 'e_done' } },
 rfsm.transition { src='ST_SET_WAYPOINT', tgt='ST_SET_WAYPOINT', events={ 'e_error' } },
 rfsm.transition { src='ST_SET_WAYPOINT', tgt='ST_REACH_FINAL', events={ 'e_done' } },
 rfsm.transition { src='ST_REACH_FINAL', tgt='ST_REACH_FINAL', events={ 'e_error' } },
 rfsm.transition { src='ST_REACH_FINAL', tgt='ST_GO_ON_2', events={ 'e_done' } },
 rfsm.transition { src='ST_GO_ON_2', tgt='ST_GO_ON_2', events={ 'e_error' } },
 rfsm.transition { src='ST_GO_ON_2', tgt='ST_CLOSE_HAND', events={ 'e_go' } },
 rfsm.transition { src='ST_GO_ON_2', tgt='ST_LOOK_IN_FRONT', events={ 'e_stop' } },
 rfsm.transition { src='ST_CLOSE_HAND', tgt='ST_OPEN_FIRST_HAND', events={ 'e_done' } },
 rfsm.transition { src='ST_OPEN_FIRST_HAND', tgt='ST_LOOK_IN_FRONT', events={ 'e_done' } },
 rfsm.transition { src='ST_LOOK_IN_FRONT', tgt='ST_SET_WAYPOINT_BACK', events={ 'e_done' } },
 rfsm.transition { src='ST_SET_WAYPOINT_BACK', tgt='ST_GO_HOME', events={ 'e_done' } },
 rfsm.transition { src='ST_GO_HOME', tgt='ST_FINI', events={ 'e_done' } },
































 }
