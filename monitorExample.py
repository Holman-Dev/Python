import sys
import time
import xpc
import PID
import math
from datetime import datetime, timedelta
# Test commit

update_interval = 1/60 # seconds
start = datetime.now()
last_update = start
# defining the initial PID values
P1 = 0.01 # PID library default = 0.2
I1 = 0.01 # default = 0
D1 = .0075 # default = 0

P2= 0.05 # PID library default = 0.2
I2 = P2/2 # default = 0
D2 = .0075 # default = 0

P3= 0.055 # PID library default = 0.2
I3 = P3/8 # default = 0
D3 = .0075 # default = 0

P4= 0.01 # PID library default = 0.2
I4 = 0.0085 # default = 0
D4 = .0085 # default = 0

P5= 0.011 # PID library default = 0.2
I5 = P5/8 # default = 0
D5 = .0075 # default = 0

# initializing PID controllers
roll_PID = PID.PID(P1, I1, D1)
Takeoff_PID = PID.PID(P2, I2, D2)
pitch_PID = PID.PID(P3, I3, D3)
waypointRoll_PID = PID.PID(P4, I4, D4)
waypointPich_PID = PID.PID(P5, I5, D5)

########################## START CODE TESTING MODES #####################################
troubleshoot_position = False
Leveler = False
Takeoff_Code = True
WayPoint_Code = False

########################## END CODE TESTING MODES #######################################

Waypoints = [45.57593528148754, -122.50271027559943, 1020, 
             45.62826770774586, -122.57372517220868, 1020, 
             45.61583265797959, -122.6435420347307, 1020, 
             45.60356804100511, -122.63269575473953, 1020, 
             45.59503418262268, -122.62136339451088, 500, 
             45.588514786909066, -122.60451261294489, 350]






def monitor():
    global last_update
    with xpc.XPlaneConnect() as client:
        posi = client.getPOSI()
        current_heading = posi[8]
        Takeoff_heading = current_heading
        Takeoff_PID.SetPoint = Takeoff_heading

        altitudedref = b"sim/cockpit/pressure/cabin_altitude_actual_ft"
        GroundAlt = client.getDREF(altitudedref)
        i = 0
        j = 0
        k = 0
        r = 0
        lat = 0
        long = 1
        alt = 2
        f = 0
        while True:
            #client.sendPOSI([-998, -998, 10000, -998, -998, -998, -998])
            while i > 0 : #LEVELER
                desired_roll = 25
                posi = client.getPOSI()
                current_heading = posi[8]
                if Takeoff_heading+175 < current_heading or k == 1:
                    desired_roll = 0
                    k = 1
                desired_altitude = 1020.2
                roll_PID.SetPoint = desired_roll
                pitch_PID.SetPoint = desired_altitude
                if (datetime.now() > last_update + timedelta(milliseconds = update_interval * 1000)):
                    last_update = datetime.now()
                    print(f"loop start - {datetime.now()}")

                    posi = client.getPOSI()
                    ctrl = client.getCTRL()
                    current_roll = posi[7]
                    altitudedref = b"sim/cockpit/pressure/cabin_altitude_actual_ft"
                    altitude = client.getDREF(altitudedref)
                    current_altitude = altitude[0]
                    roll_PID.update(current_roll)
                    pitch_PID.update(current_altitude)
                    new_ail_ctrl = roll_PID.output
                    new_ele_ctrl = pitch_PID.output
                    ctrl = [new_ele_ctrl, new_ail_ctrl, -998, -998,-998, -998, -998] # ele, ail, rud, thr. -998 means don't change
                    client.sendCTRL(ctrl)
                    output = f"current values --    roll: {current_roll: 0.3f},  pitch: {current_pitch: 0.3f}"
                    output = output + "\n" + f"PID outputs    --    roll: {roll_PID.output: 0.3f},  pitch: {pitch_PID.output: 0.3f}"
                    output = output + "\n"
                    print(output)
                
                
            while Takeoff_Code and j == 0:
                if r == 0: # Turn off brakes once
                    r = 1
                    client.sendDREF(b"sim/cockpit2/controls/parking_brake_ratio",0)
                desired_roll = 0
                desired_pitch = 10
                roll_PID.SetPoint = desired_roll
                pitch_PID.SetPoint = desired_pitch
                if (datetime.now() > last_update + timedelta(milliseconds = update_interval * 1000)):
                    last_update = datetime.now()
                    #print(f"loop start - {datetime.now()}")

                    posi = client.getPOSI()
                    ctrl = client.getCTRL()
                    current_heading = posi[8]
                    
                    Takeoff_PID.update(current_heading)
                    
                    new_rudder_ctrl = Takeoff_PID.output
                    
                    ctrl = [-998, -998, new_rudder_ctrl, 1,-998, -998, -998] # ele, ail, rud, thr. -998 means don't change
                    client.sendCTRL(ctrl)
                    
                    airspeeddref = b"sim/flightmodel/position/indicated_airspeed"
                    airspeed = client.getDREF(airspeeddref)

                    altitudedref = b"sim/cockpit/pressure/cabin_altitude_actual_ft"
                    altitude = client.getDREF(altitudedref)


                    if airspeed[0] > 70 and altitude[0]-GroundAlt[0] < 500:
                        posi = client.getPOSI()
                        ctrl = client.getCTRL()
                        current_roll = posi[7]
                        current_pitch = posi[6]
                        roll_PID.update(current_roll)
                        pitch_PID.update(current_pitch)
                        new_ail_ctrl = roll_PID.output
                        new_ele_ctrl = pitch_PID.output
                        ctrl = [new_ele_ctrl, new_ail_ctrl, -998, -998,-998, -998, -998] # ele, ail, rud, thr. -998 means don't change
                        client.sendCTRL(ctrl)
                    if altitude[0]-GroundAlt[0] > 500:
                        j = 1
                        

            while j==1: # Waypoints start after Takeoff to 500 ft
                desired_Lat = Waypoints[lat]
                desired_Long = Waypoints[long]
                desired_altitude = Waypoints[alt]
                if (datetime.now() > last_update + timedelta(milliseconds = update_interval * 1000)):
                    last_update = datetime.now()
                    #print(f"loop start - {datetime.now()}")
                    posi = client.getPOSI()
                    current_heading = posi[8]
                    altitudedref = b"sim/cockpit/pressure/cabin_altitude_actual_ft"
                    altitude = client.getDREF(altitudedref)
                    latitude = b"sim/flightmodel/position/latitude"
                    current_Lat = client.getDREF(latitude)
                    longitude = b"sim/flightmodel/position/longitude"
                    current_Long = client.getDREF(longitude)

                    ### The following if statements determine what "quadrant, 1,2,3, or 4" the plane is in, then returns the heading needed to fly towards waypoint ###
                   
                    if current_Lat[0] > desired_Lat and current_Long[0] > desired_Long: # Quadrant 1
                        theta = math.degrees(math.atan(abs(desired_Long-current_Long[0])/abs(desired_Lat-current_Lat[0])))
                        desired_Head = theta + 180
                        #print("You're in Quadrant 1")
                    elif current_Lat[0] < desired_Lat and current_Long[0] > desired_Long: # Quadrant 2
                        theta = math.degrees(math.atan(abs(desired_Lat-current_Lat[0])/abs(desired_Long-current_Long[0])))
                        desired_Head = theta + 270
                        #print("You're in Quadrant 2")
                    elif current_Lat[0] < desired_Lat and current_Long[0] < desired_Long: # Quadrant 3
                        theta = math.degrees(math.atan(abs(desired_Long-current_Long[0])/abs(desired_Lat-current_Lat[0])))
                        desired_Head = theta
                        #print("You're in Quadrant 3")
                    else: # Quadrant 4
                        theta = math.degrees(math.atan(abs(desired_Lat-current_Lat[0])/abs(desired_Long-current_Long[0])))
                        desired_Head = theta + 90
                        #print("You're in Quadrant 4")
                        

                    ### Final section of code for Waypoint_Code controls the elevator and alierons via PID to correct position ###
                    ### To ensure reasonable decent/acent rates, if the difference in alt > 50, pitch up or down 10 degrees ###
                    if abs(altitude[0]-desired_altitude) > 50:
                        print("Saftey Pitch Mode")
                        if altitude[0] < desired_altitude:
                            desired_pitch = 10
                        if altitude[0] > desired_altitude:
                            desired_pitch = -10
                        pitch_PID.SetPoint = desired_pitch
                        current_pitch = posi[6]
                        print(current_pitch)
                        print(desired_pitch)
                        pitch_PID.update(current_pitch)
                        new_ele_ctrl = pitch_PID.output
                    else:
                        waypointPich_PID.SetPoint = desired_altitude
                        waypointPich_PID.update(altitude[0])
                        new_ele_ctrl = waypointPich_PID.output
                    ### To ensure reasonable roll rates, if the difference in heading > 20, roll 20 degrees ###
                    if abs(current_heading - desired_Head) > 40:
                        print("Saftey Roll Mode")
                        desired_roll = -35
                        roll_PID.SetPoint = desired_roll
                        current_roll = posi[7]
                        roll_PID.update(current_roll)
                        new_ail_ctrl = roll_PID.output
                    else:
                        waypointRoll_PID.SetPoint = desired_Head
                        waypointRoll_PID.update(current_heading)
                        new_ail_ctrl = waypointRoll_PID.output
                    ### Send Control to Waypoint ###
                    ctrl = [new_ele_ctrl, new_ail_ctrl, -998, -998,-998, -998, -998] # ele, ail, rud, thr. -998 means don't change
                    client.sendCTRL(ctrl)
                    distance_to_desired = client.distance(current_Lat[0], desired_Lat, current_Long[0], desired_Long) # Distance to next waypoint in feet
                    print("Distance To Next Waypoint: ", distance_to_desired)
                    if distance_to_desired < 5000: # Distance in Feet
                        lat = lat + 3
                        long = long + 3
                        alt = alt + 3
                        k = k + 1
                        f = 0
                        if k == 2:
                             ctrl = [-998, -998, -998, 0.75,-998, -998, -998]
                             j = 0 # This breaks out of waypoint mode and enters aircraft into FREE FLIGHT Mode. Waypoint Navigation will resume after free flight time limit
                        if k == 3:
                             ctrl = [-998, -998, -998, 0.2,-998, -998, -998]
                        if k == 5:
                             ctrl = [-998, -998, -998, 0.1,-998, -998, -998]
                    client.sendCTRL(ctrl)
                    if alt+1 > len(Waypoints): # Code Stops After all Waypoints have been reached
                        break

            while j == 0 and k == 2: # FREE FLIGHT MODE: Time limited mode
                if f == 0:
                    FF_Start_Time = datetime.now()
                    print("You have entered FREE FLIGHT MODE! Control the aircraft as you wish!")
                    time.sleep(2)
                    f = f + 1
                if (datetime.now() > FF_Start_Time + timedelta(milliseconds = 60 * 1000)): # If one minute has passed while in Free Flight Mode, Free Flight Ends and Waypoint Nav Resumes
                    j = 1 # Breaks out of Free Flight after current loop and Allows Waypoint Nav to Resume
                Free_Flight_Origin = [Waypoints[lat-3], Waypoints[long-3], Waypoints[alt-3]]
                Origin_Lat = Free_Flight_Origin[0]
                Origin_Long = Free_Flight_Origin[1]
                Origin_Alt = Free_Flight_Origin[2]
                Free_Flight_Radius = 4000 # Free Flight Zone Radius in Feet
                Free_Flight_Floor = GroundAlt[0]+750 # Free Flight Floor (Lower Altitude Limit) in Feet
                Free_Flight_Ceiling = GroundAlt[0]+3000 # Free Flight Ceiling (Upper Altitude Limit) in Feet
                if (datetime.now() > last_update + timedelta(milliseconds = update_interval * 1000)):
                    last_update = datetime.now()
                    posi = client.getPOSI()
                    current_heading = posi[8]
                    altitudedref = b"sim/cockpit/pressure/cabin_altitude_actual_ft"
                    altitude = client.getDREF(altitudedref)
                    latitude = b"sim/flightmodel/position/latitude"
                    current_Lat = client.getDREF(latitude)
                    longitude = b"sim/flightmodel/position/longitude"
                    current_Long = client.getDREF(longitude)
                    distance_to_boundary = Free_Flight_Radius-client.distance(current_Lat[0], Origin_Lat, current_Long[0], Origin_Long) # Distance to Boundary of Free Flight Zone in feet
                    if distance_to_boundary < 1000:
                        print("Approaching Edge Of Free Flight Zone!!!", "\n", "Distance to Boundary = ", distance_to_boundary)
                    if altitude[0] < Free_Flight_Floor+200:
                        print("Too LOW!!! PULL UP!!!")
                    elif altitude[0] > Free_Flight_Ceiling-200:
                        print("Too HIGH!!! NOSE DOWN!!!")
                    



if __name__ == "__main__":
    monitor()