"""
Description:

The purpose of the function is to integrate displacement using the linear velocities
and time received by the dvl and finally to return the displacement.

This function will receive three lists:
    In the first list we receive initial lineal velocity(x,y,z).
    In the second list we receive final lineal velocity(x,y,z).
    In the third list we will receive initial time (x) and final time (y)

We know that if we want to calculate the displacement from the data given by
the dvl we have to use the formula:

displacement = V * T

We also use the formula three times for x,y,z direction.

Args:

iVelocity: List that represents the x,y and z Initial Velocity

fVelocity: List that represents x,y and z finals velocities

time: List has two elements, the first element is the initial time and the second element is the final velocity.

Author:

Ale Pagan Andujar

"""

   
        

def displacement(iVelocity, fVelocity, time):
    """This function expects three lists as parameters, the first parameter must be the
    initial velocity vector, second parameter must be the final velocity vector. The time
    parameter must have: time[0] = initial time and time[1] = final time. This function
    will return a list that represents the displacement in x,y and z.
    """

    change_velx = fVelocity[0] - iVelocity[0]
    """
    This variable will hold the rate of change of velocity in the x direction.
    """

    change_vely = fVelocity[1] - iVelocity[1]
    """
    This variable will hold the rate of change in velocity in the y direction.
    """

    change_velz = fVelocity[2] - iVelocity[2]
    """
    This variable will hold the rate of change in velocity in the z direction.
    """

    change_time = time[1] - time[0]
    """
    This variable will hold the rate of change in time from the given velocity.
    """

    displacement = [0] * 3
    """
    This list will store the displacement in x,y,z respectively.
    """

    displacement[0] = change_velx * change_time
    displacement[1] = change_vely * change_time
    displacement[2] = change_velz * change_time
    return displacement