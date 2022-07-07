data is on folder cleaned_1Intermediate and cleaned_2Intermediate

To navigate through the directory, first choose either 1 or 2 intermediate, then choose path number(different path number will have different start,intermediate, or destination point). Then choose the wind speed(The number there is our fan power settings). lastly choose wind_direction(the number here is the global wind direction. Look below for more details on global wind direction).

column on csv files for Intermediate:

1. time.ms -> showing time that has passed in milliseconds

2. stateEstimate.x -> showing current estimation for x coordinate

3. stateEstimate.y -> showing current estimation for y coordinate

4. stateEstimate.z -> showing current estimation for z coordinate

5. stabilizer.roll -> showing current estimation for roll

6. stabilizer.pitch -> showing current estimation for pitch

7. stabilizer.yaw -> showing current estimation for yaw

8. pm.vbat -> showing current estimation for voltage

9. drone_id -> showing the id of drone with this csv. Possible drone ids are 0,1,and 2

10. node_name -> it will either show the name of the node e.g. A1,B2,etc. or it will say "fly" if it is currently not in the ground.

11. wind_speed -> in km/hours

12. global_wind_direction -> following quadran, so wind going to east is 0, wind going to north is 90, west is 180 south is 270. None if there is no wind

13. relative_wind_direction -> global wind direction - drone flight path angle.
drone flight path angle is also following quadran. So if the drone is moving from west to east, the angle will be 0. if the drone is moving from south to north the angle will be 90. and so on. None if there is no wind.

14. travel_dist -> distance traveled so far. in meter

15. waiting_time -> the format is [waitingTimeInStartNode|waitingTimeInIntermediate|waitingTimeInDestination]. in milliseconds.

16. curr_node_role -> Tell the role of the node the drone is currently in. For 1 intermediate possible roles are start, intermediate, and destination. For 2 intermediate possible roles are start, first intermediate, second intermediate, and destination. If the drone is not currently in the ground then it will says fly.
