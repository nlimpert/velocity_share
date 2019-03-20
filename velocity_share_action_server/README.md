# velocity_share_action_server
This package provides an action_server which has the following purpose:
1. For any new goal received, forward it to the move_base action server.
2. In case the plan calculated by the currently used global planner within move_base leads through an area that is obstructed given the information form the velocity_share_layer, do the following:
  2.1. Stop right at the point which is close to the obstruction area.
  2.2. Wait until the area gets free and continue with the original goal (namely continue with step 1.)
3. Else if no obstruction occurs just behave like move_base and set itself to the appropriate actionserver states as reported by move_base.

Note: Step 2 shall be implemented as an action that can be exchanged with other actions. E.g. in addition to this action another option might be to try to move behind the approaching robot, instead of waiting.
