data is on folder 1Internediate and 2Intermediate

column on csv files for 1Intermediate:

1. time.ms -> showing time that has passed in milliseconds

2. stateEstimate.x -> showing current estimation for x coordinate

3. stateEstimate.y -> showing current estimation for y coordinate

4. stateEstimate.z -> showing current estimation for z coordinate

5. stabilizer.roll -> showing current estimation for roll

6. stabilizer.pitch -> showing current estimation for pitch

7. stabilizer.yaw -> showing current estimation for yaw

8. pm.vbat -> showing current estimation for voltage

9. drone_id -> showing the id of drone with this csv. Possible drone ids are 0,1,and 2

10. node_name -> don't use

11. wind_speed 

12. global_wind_direction -> following quadran, so east wind is 0, north wind is 90, west is 180 south is 270

13. relative_wind_direction -> global wind direction - drone flight path angle.
drone flight path angle is also following quadran. So if the drone is moving from west to east, the angle will be 0. if the drone is moving from south to north the angle will be 90. and so on

14. travel_dist -> distance traveled so far. in meter

15. waiting_time -> the format is [waitingTimeInStartNode|waitingTimeInIntermediate|waitingTimeInDestination]. in milliseconds.