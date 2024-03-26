def find_grid_location(Lab_Layout,Frame):
    for i, sublist in enumerate(Lab_Layout):
        # Check if the element is in the sublist
        if Frame in sublist:
            # Find the column index of the element in the sublist
            j = sublist.index(Frame)
            # Store both row and column indexes in location
            location = (i, j)
            return location
    return (-1,-1)

Frame_Km = "Km"
Frame_Encoder = "Encoder"

Frame_Pitch = "Pitch"

Frame_Roll = "Roll"
Frame_Action = "Action"
Frame_Yaw = "Yaw"
Frame_Control = "Control"

Frame_Empty = "Empty"

Tune_km = [[Frame_Km],[Frame_Encoder]]                                                      # Lab H4
Tune_Pitch = [[Frame_Km],[Frame_Pitch],[Frame_Encoder]]                                     # Lab H7
Tune_Lat = [[Frame_Km,Frame_Action],[Frame_Roll, Frame_Yaw],[Frame_Encoder,Frame_Control]]  # Lab H8
Tune_Full = [[Frame_Km,Frame_Empty,Frame_Action],[Frame_Roll, Frame_Yaw, Frame_Pitch],[Frame_Encoder,Frame_Empty,Frame_Control]]  # Lab H9