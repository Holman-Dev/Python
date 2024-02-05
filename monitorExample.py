import sys
import time
import xpc
import PID
import math
from datetime import datetime, timedelta
from utility import interpolate_coordinates
from plotting import plot_course

# Test commit

update_interval = 1/60 # seconds
start = datetime.now()
last_update = start
# defining the initial PID values
P1 = 0.01 # PID library default = 0.2
I1 = 0.01 # default = 0
D1 = .0075 # default = 0

P2= 0.035 # PID library default = 0.2
I2 = P2/2 # default = 0
D2 = .0195 # default = 0

P3= 0.035 # PID library default = 0.2
I3 = P3/6 # default = 0
D3 = .0075 # default = 0

P4= 0.0075 # PID library default = 0.2
I4 = 0.001 # default = 0
D4 = 0.02 # default = 0

P5= 0.01 # PID library default = 0.2
I5 = 0.005 # default = 0
D5 = .025 # default = 0

# initializing PID controllers
roll_PID = PID.PID(P1, I1, D1)
Takeoff_PID = PID.PID(P2, I2, D2)
pitch_PID = PID.PID(P3, I3, D3)
waypointRoll_PID = PID.PID(P4, I4, D4)
waypointPich_PID = PID.PID(P5, I5, D5, 0.55)

########################## START CODE TESTING MODES #####################################
troubleshoot_position = False
Leveler = False
Takeoff_Code = True
WayPoint_Code = False

########################## END CODE TESTING MODES #######################################

Waypoints = [45.57612160483264, -122.57113385513277, 750,
             45.57593528148754, -122.50271027559943, 1020,
             45.605139923276205, -122.52885522013545, 1020, 
             45.62826770774586, -122.57372517220868, 750,
             45.62855246381241, -122.64928064214199, 550, 
             45.604401980338814, -122.64484789761462, 250, 
             45.59886843478141, -122.63103059742295, 120, 
             45.593722594311856, -122.61763971854334, 50]

#Bimini Waypoints
#Waypoints = [25.69424, -79.23323, 2000, 
             #25.67588, -79.26742, 2000, 
             #25.68703, -79.31143, 2000, 
             #25.70122, -79.27606, 50]

Free_Flight_Waypoint = [ 45.62826770774586, -122.57372517220868, 1020] # Must be EXACTLY one of the original waypoints in Waypoints array
Waypoints = interpolate_coordinates(Waypoints) # Cubic Spline interpolation to increase waypoint resolution
Total_Points = len(Waypoints)
Number_OF_Waypoints = int(len(Waypoints)/3)

#print(Total_Points)
#print(Number_OF_Waypoints)
#print(Waypoints[0], Waypoints[Number_OF_Waypoints], Waypoints[Number_OF_Waypoints*2])

# Your specified latitude, longitude, and radius
center_lat = Free_Flight_Waypoint[0]  # replace with your latitude
center_lon = Free_Flight_Waypoint[1]  # replace with your longitude
radius = 0.8  # replace with your radius in miles




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
        k_Free = 0
        r = 0
        lat = 0
        long = Number_OF_Waypoints
        alt = Number_OF_Waypoints*2
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


                    if airspeed[0] > 70 and altitude[0] < 500: #altitude[0]-GroundAlt[0]
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
                    if altitude[0] > 500: #altitude[0]-GroundAlt[0]
                        j = 1
                        

            while j==1: # Waypoints start after Takeoff to 500 ft
                if alt+1 > len(Waypoints): # Code Stops After all Waypoints have been reached
                        break
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
                        Q = 3
                        #print("You're in Quadrant 3")
                    else: # Quadrant 4
                        theta = math.degrees(math.atan(abs(desired_Lat-current_Lat[0])/abs(desired_Long-current_Long[0])))
                        desired_Head = theta + 90
                        Q = 4
                        #print("You're in Quadrant 4")
                    desired_Head = desired_Head % 360
                    current_heading = current_heading % 360
                    shortest_angular_difference = -(((desired_Head - current_heading + 180) % 360) - 180)
                    print("Angular Difference: ",shortest_angular_difference)
                    ### Final section of code for Waypoint_Code controls the elevator and alierons via PID to correct position ###
                    ### To ensure reasonable decent/acent rates, if the difference in alt > 50, pitch up or down 10 degrees ###
                    if abs(altitude[0]-desired_altitude) > 500:
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
                    waypointRoll_PID.SetPoint = 0
                    waypointRoll_PID.update(shortest_angular_difference)
                    new_ail_ctrl = waypointRoll_PID.output
                    Slip = client.getDREF(b"sim/cockpit2/gauges/indicators/slip_deg")
                    Takeoff_PID.SetPoint = 0
                    Takeoff_PID.update(Slip[0])
                    new_rudder_ctrl = Takeoff_PID.output
                    ### Send Control to Waypoint ###
                    ctrl = [new_ele_ctrl, new_ail_ctrl, new_rudder_ctrl, -998,-998, -998, -998] # ele, ail, rud, thr. -998 means don't change
                    client.sendCTRL(ctrl)
                    distance_to_desired = client.distance(current_Lat[0], desired_Lat, current_Long[0], desired_Long) # Distance to next waypoint in feet
                    print("Distance To Next Waypoint: ", distance_to_desired)
                    if distance_to_desired < 1200: # Distance in Feet
                        lat = lat + 1
                        long = long + 1
                        alt = alt + 1
                        k = k + 1
                        if Free_Flight_Waypoint[0] == Waypoints[lat] and Free_Flight_Waypoint[1] == Waypoints[long]:
                            k_Free = k+1
                        f = 0
                        '''if k == 20:
                            j = 0'''
                        if k > Number_OF_Waypoints*0.75:
                             ctrl = [-998, -998, -998, 0.75,-998, -998, -998]
                             #j = 0 # This breaks out of waypoint mode and enters aircraft into FREE FLIGHT Mode. Waypoint Navigation will resume after free flight time limit
                        if k > Number_OF_Waypoints*0.8:
                             ctrl = [-998, -998, -998, 0.5,-998, -998, -998]
                        if k > Number_OF_Waypoints*0.9:
                             ctrl = [-998, -998, -998, 0.25,-998, -998, -998]
                        if k > Number_OF_Waypoints*0.95:
                             ctrl = [-998, -998, -998, 0.05,-998, -998, -998]
                    client.sendCTRL(ctrl)

            while j == 0 and k == 10: # FREE FLIGHT MODE: Time limited mode
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

                    plot_course(center_lat, center_lon, radius, current_Lat[0], current_Long[0])

                    distance_to_boundary = Free_Flight_Radius-client.distance(current_Lat[0], Origin_Lat, current_Long[0], Origin_Long) # Distance to Boundary of Free Flight Zone in feet
                    if distance_to_boundary < 1000:
                        print("Approaching Edge Of Free Flight Zone!!!", "\n", "Distance to Boundary = ", distance_to_boundary)
                    if altitude[0] < Free_Flight_Floor+200:
                        print("Too LOW!!! PULL UP!!!")
                    elif altitude[0] > Free_Flight_Ceiling-200:
                        print("Too HIGH!!! NOSE DOWN!!!")
                    



if __name__ == "__main__":
    monitor()