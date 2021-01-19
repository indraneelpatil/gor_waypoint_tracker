"""

Path planning with Bezier curve.

author: Atsushi Sakai(@Atsushi_twi)

"""

import matplotlib.pyplot as plt
import numpy as np
import scipy.special
import math
#import quintic_polynomials_planner as quintic

show_animation = True

def calc_8points_bezier_path(sx, sy, syaw, ex, ey, eyaw):
    c1 = 7.2364
    c2 = 0.4*(pow(6,0.5)-1)
    c3 = (c2 + 4)/(c1 + 6)
    
    kmax = 1

    step_size = sx - ex
    temp=(step_size) *0.5
    midx = temp
    midy = 0.5+(sy - ey)*0.5

    vector_1 = [midx-sx,midy-sy]
    vector_2 = [ex-midx,ey-midy]
    vector_3 = [ex-sx,ey-sy]

    print("Vector 1 is :",midx-sx," ",midy-sy)


    unit_vector_1 = vector_1 / np.linalg.norm(vector_1)
    print("Unit Vector 1 is :",unit_vector_1[0]," ",unit_vector_1[1])
    unit_vector_2 = vector_2 / np.linalg.norm(vector_2)
    dot_product = np.dot(unit_vector_1, unit_vector_2)
    print("dot product is:",dot_product)
    sigma = np.arccos((dot_product))
    print("Angle between vectors is :",math.degrees(sigma))
    beta = sigma/2

    d = (pow(c2+4,2)/(54*c3))*(np.sin(beta)/(kmax*pow(np.cos(beta),2)))
    #d = (step_size) *0.5

    #hb = ((c2+4)*vector_3[1]*pow(np.cos(sigma-beta),2)*np.sin(beta))/(np.cos(beta)*np.sin(sigma)*((c1-6)*np.cos(beta)*np.sin(sigma-beta)+6*np.sin(sigma)))
    #he = (hb*pow(np.cos(beta),2)*np.sin(sigma-beta))/(np.sin(beta)*pow(np.cos(sigma-beta),2))
    hb = c3*d
    he = c3*d

    print(d,hb,he)

    gb=c2*hb
    ge=c2*he


    kb = (6/(c2+4))*hb*np.cos(beta)
    ke = (6/(c2+4))*he*np.cos(sigma-beta)
    

    B0x = d*(-1*unit_vector_1[0]) +  midx
    B0y = d*(-1*unit_vector_1[1]) +  midy

    B1x = B0x - gb*(-1*unit_vector_1[0])
    B1y = B0y - gb*(-1*unit_vector_1[1])

    B2x = B1x - hb*(-1*unit_vector_1[0])
    B2y = B1y - hb*(-1*unit_vector_1[1])

    E0x = midx + d*unit_vector_2[0]
    E0y = midy + d*unit_vector_2[1]

    E1x = E0x - ge*unit_vector_2[0]
    E1y = E0y - ge*unit_vector_2[1]

    E2x = E1x - he*unit_vector_2[0]
    E2y = E1y - he*unit_vector_2[1]

    vector_4 =[B2x-E2x,B2y-E2y]
    unit_vector_4 = vector_4 / np.linalg.norm(vector_4)

    B3x = B2x + kb*unit_vector_4[0]
    B3y = B2y + kb*unit_vector_4[1]

    E3x = E2x - ke*unit_vector_4[0]
    E3y = E2y - ke*unit_vector_4[1]

    control_points_1 = np.array(
        [[B0x, B0y],
        [B1x, B1y],
        [B2x, B2y],
        [B3x, B3y]])
    
    control_points_2 = np.array(
        [[E0x, E1y],
        [E1x, E1y],
        [E2x, E2y],
        [E3x, E3y]])
       
    return control_points_1,control_points_2



def calc_4points_bezier_path(sx, sy, syaw, ex, ey, eyaw, offset):
    """
    Compute control points and path given start and end position.

    :param sx: (float) x-coordinate of the starting point
    :param sy: (float) y-coordinate of the starting point
    :param syaw: (float) yaw angle at start
    :param ex: (float) x-coordinate of the ending point
    :param ey: (float) y-coordinate of the ending point
    :param eyaw: (float) yaw angle at the end
    :param offset: (float)
    :return: (numpy array, numpy array)
    """
    dist = 0.5* np.sqrt((sx - ex) ** 2 + (sy - ey) ** 2) #/ offset
    control_points = np.array(
        [[sx, sy],
         [sx + dist * np.cos(syaw), sy + dist * np.sin(syaw)],
         [ex - dist * np.cos(eyaw), ey - dist * np.sin(eyaw)],
         [ex, ey]])
    
    '''
    step_size = sx - ex
    temp=(step_size) *0.5
    control_points = np.array(
        [[sx, sy],
         [temp, sy + (temp* np.tan(syaw))],
         [(step_size) *0.5, ((step_size) *0.5* np.tan(eyaw))],
         [ex, ey]])
    
    bezier_ctrl_point[1].y = (0.5 * (move_data[move_data_current_step_idx].step_size + local_deltaY));
        bezier_ctrl_point[1].x = local_deltaX + (bezier_ctrl_point[1].y * tan(local_theta));

        bezier_ctrl_point[2].y = (0.5 * (move_data[move_data_current_step_idx].step_size + local_deltaY));
        bezier_ctrl_point[2].x = 0.0;
    '''
    path = calc_bezier_path(control_points, n_points=100)

    return path, control_points


def calc_bezier_path(control_points, n_points=100):
    """
    Compute bezier path (trajectory) given control points.

    :param control_points: (numpy array)
    :param n_points: (int) number of points in the trajectory
    :return: (numpy array)
    """
    traj = []
    for t in np.linspace(0, 1, n_points):
        traj.append(bezier(t, control_points))

    return np.array(traj)

# 3*((d-3*c+3*b-a)*t^2+2*(c-2*b+a)*t+(b-a))
def bezier_first_deriv_poly(t,control_points):
    x_t_prime = 3*(pow(t,2)*(control_points[3][0]-3*control_points[2][0]+3*control_points[1][0]-control_points[0][0])
                + 2*(control_points[2][0]-2*control_points[1][0]+control_points[0][0])*t + 
                control_points[1][0] -control_points[0][0] )
    y_t_prime = 3*(pow(t,2)*(control_points[3][1]-3*control_points[2][1]+3*control_points[1][1]-control_points[0][1])
                + 2*(control_points[2][1]-2*control_points[1][1]+control_points[0][1])*t + 
                control_points[1][1] -control_points[0][1] ) 

    return pow(pow(x_t_prime,2)+pow(y_t_prime,2),0.5)          


def bernstein_poly(n, i, t):
    """
    Bernstein polynom.

    :param n: (int) polynom degree
    :param i: (int)
    :param t: (float)
    :return: (float)
    """
    return scipy.special.comb(n, i) * t ** i * (1 - t) ** (n - i)


def bezier(t, control_points):
    """
    Return one point on the bezier curve.

    :param t: (float) number in [0, 1]
    :param control_points: (numpy array)
    :return: (numpy array) Coordinates of the point
    """
    n = len(control_points) - 1
    return np.sum([bernstein_poly(n, i, t) * control_points[i] for i in range(n + 1)], axis=0)


def bezier_derivatives_control_points(control_points, n_derivatives):
    """
    Compute control points of the successive derivatives of a given bezier curve.

    A derivative of a bezier curve is a bezier curve.
    See https://pomax.github.io/bezierinfo/#derivatives
    for detailed explanations

    :param control_points: (numpy array)
    :param n_derivatives: (int)
    e.g., n_derivatives=2 -> compute control points for first and second derivatives
    :return: ([numpy array])
    """
    w = {0: control_points}
    for i in range(n_derivatives):
        n = len(w[i])
        w[i + 1] = np.array([(n - 1) * (w[i][j + 1] - w[i][j])
                             for j in range(n - 1)])
    return w


def curvature(dx, dy, ddx, ddy):
    """
    Compute curvature at one point given first and second derivatives.

    :param dx: (float) First derivative along x axis
    :param dy: (float)
    :param ddx: (float) Second derivative along x axis
    :param ddy: (float)
    :return: (float)
    """
    return (dx * ddy - dy * ddx) / (dx ** 2 + dy ** 2) ** (3 / 2)


def plot_arrow(x, y, yaw, length=0.1, width=0.05, fc="r", ec="k"):  # pragma: no cover
    """Plot arrow."""
    if not isinstance(x, float):
        for (ix, iy, iyaw) in zip(x, y, yaw):
            plot_arrow(ix, iy, iyaw)
    else:
        plt.arrow(x, y, length * np.cos(yaw), length * np.sin(yaw),
                  fc=fc, ec=ec, head_width=width, head_length=width)
        plt.plot(x, y)

def bezier_length(control_points):
    a = np.array((control_points[0][0] ,control_points[0][1]))
    b = np.array((control_points[1][0] ,control_points[1][1]))
    c = np.array((control_points[2][0] ,control_points[2][1]))
    d = np.array((control_points[3][0] ,control_points[3][1]))

    chord_length=  np.linalg.norm(a-d)
    net_1=  np.linalg.norm(a-b)
    net_2=  np.linalg.norm(b-c)
    net_3=  np.linalg.norm(c-d)

    return (chord_length+net_1+net_2+net_3)/2

def bezier_length_linear_approx(control_points):
    b_path = calc_bezier_path(control_points, n_points=5000)    
    total_length=0
    #print b_path
    it=0
    for i in b_path:
        if it<(5000-1):
            a = np.array((b_path[it][0] ,b_path[it][1]))
            b = np.array((b_path[it+1][0] ,b_path[it+1][1]))
            total_length=total_length+ np.linalg.norm(a-b)
            it=it+1
    print it

    return total_length




    
#Bezier curve - > x, y, theta domain
# linear velocity profile - > time domain


def main():
    """ Common parameters """
    angular_velocity_time_interval=0.1 #s
    start_x = 10.0  # [m]
    start_y = 1.0  # [m]
    start_yaw = np.radians(190.0)  # [rad]

    end_x = -0.0  # [m]
    end_y = -0.0  # [m]
    end_yaw = np.radians(180.0)  # [rad]

    """ Quintic Parameters """
    
    def_accel_dist=1.9
    def_decel_dist=1.9
    sv = 0.06  # start speed [m/s]
    sa = 0.0  # start accel [m/ss]
    gv = 1.0  # goal speed [m/s]
    ga = 0.1  # goal accel [m/ss]
    max_accel = 0.59  # max accel [m/ss]
    max_jerk = 1  # max jerk [m/sss]
    max_vel= 1.2
    dt = 0.1  # time tick [s]

    """ 1st linear profile """
    s_1v = sv
    s_1a = 0.0
    g_1v=max_vel
    g_1a=0
    s_1x=start_x
    s_1y=0.0
    s_1yaw=np.deg2rad(180.0)
    g_1x=s_1x-def_accel_dist
    g_1y=0
    g_1yaw=np.deg2rad(180.0)
    time, x, y, yaw, v, a, j = quintic.quintic_polynomials_planner(
        s_1x, s_1y, s_1yaw, s_1v, s_1a, g_1x, g_1y, g_1yaw, g_1v, g_1a, max_accel, max_jerk, dt)

    """ 2nd linear profile: complete journey at top speed """
    total_distance = abs(end_x-start_x)
    distance_top_speed=total_distance-3.8
    seconds_top_speed=distance_top_speed/max_vel
    temp_len=len(time)
    
    for j in range(int(round(seconds_top_speed,1)*10)):
        time.append(time[temp_len-1]+float(j)/10)
        v.append(max_vel)
        pass

    """ 3rd linear profile """
    s_3v = max_vel
    s_3a = 0.0
    g_3v=0.0
    g_3a=0
    s_3x=end_x+1.9
    s_3y=0.0
    s_3yaw=np.deg2rad(180.0)
    g_3x=end_x
    g_3y=0
    g_3yaw=np.deg2rad(180.0)
    time_new, x_new, y_new, yaw_new, v_new, a_new, j_new = quintic.quintic_polynomials_planner(
        s_3x, s_3y, s_3yaw, s_3v, s_3a, g_3x, g_3y, g_3yaw, g_3v, g_3a, max_accel, max_jerk, dt)

    temp_len2=len(time)
    for u in range(len(time_new)):
        time.append(time[temp_len2-1]+time_new[u])
        v.append(v_new[u])
    
    print "Total time for executing linear velocity :",time[len(time)-1]
    print "Total angular velocity control signals :", time[len(time)-1]/angular_velocity_time_interval
    
    #Round off each time step in time list
    for u in range(len(time)):
        time[u]=round(time[u],1)

    if show_animation:  # pragma: no cover
        #plt.plot(x, y, "-r")
        #plt.grid(True)

        #plt.subplots()
        #plt.plot(time, [np.rad2deg(i) for i in yaw], "-r")
        #plt.xlabel("Time[s]")
        #plt.ylabel("Yaw[deg]")
        #plt.grid(True)

        plt.subplots()
        plt.plot(time, v, "-r")
        plt.xlabel("Time[s]")
        plt.ylabel("Speed[m/s]")
        plt.grid(True)

        plt.show()

    print "#########################################################"

    """ bezier curve."""
   
    offset = 2.0

    path, control_points = calc_4points_bezier_path(
        start_x, start_y, start_yaw, end_x, end_y, end_yaw, offset)

    # Note: alternatively, instead of specifying start and end position
    # you can directly define n control points and compute the path:
    # control_points = np.array([[5., 1.], [-2.78, 1.], [-11.5, -4.5], [-6., -8.]])
    # path = calc_bezier_path(control_points, n_points=100)
    
    # Find length of bezier curve
    arc_length=bezier_length(control_points)
    print "Total Arc length of bezier curve is :",arc_length
    #print control_points

    # Display the tangent, normal and radius of cruvature at a given point
    t = 0.5  # Number in [0, 1]
    x_target, y_target = bezier(t, control_points)
    print x_target,y_target
    i=0
    x_points=[]
    y_points=[]
    path_all_points=np.array([])
    #Run simulation to find angular control signals about bezier curve
    dist_travelled=0
    control_signal_count=0
    #print time
    while dist_travelled<(start_x-end_x):
        if i==0:
            x_previous, y_previous = bezier(0, control_points)
            previous_heading=-10
            i=i+1
            x_points.append(x_previous)
            y_points.append(y_previous)
            print "first point is :",x_previous,y_previous
            control_signal_count=control_signal_count+1
            #No control signal on the first point
        else:
            # Need to look up current linear velocity
            print "Looking up linear velocity at:",round(angular_velocity_time_interval*control_signal_count,1)
            linear_vel=float( v[time.index(round(angular_velocity_time_interval*control_signal_count,1))])
            control_signal_count=control_signal_count+1
            print "Linear velocity at this point:","{:.20f}".format(linear_vel)
            delta_s=angular_velocity_time_interval*linear_vel
            dist_travelled=dist_travelled+delta_s
            print dist_travelled/(start_x-end_x)
            x_current, y_current = bezier(dist_travelled/(start_x-end_x), control_points) #Point after another 100ms interval
            print "Next point is :",x_current,y_current
            x_points.append(x_current)
            y_points.append(y_current)

            tan_theta=(y_current-y_previous)/(x_current-x_previous)
            current_heading=math.degrees(math.atan(tan_theta))
            print(i,"Theta correction required :",current_heading-previous_heading)
            print("Angular velocity required degrees/sec :",float( current_heading-previous_heading)/angular_velocity_time_interval)
            previous_heading=current_heading
            x_previous=x_current
            y_previous=y_current
            print("****")


    all_points=zip(x_points,y_points)

            

    
    print "#########################################################"
    

    derivatives_cp = bezier_derivatives_control_points(control_points, 2)
    point = bezier(t, control_points)
    dt = bezier(t, derivatives_cp[1])
    ddt = bezier(t, derivatives_cp[2])
    # Radius of curvature
    radius = 1 / curvature(dt[0], dt[1], ddt[0], ddt[1])
    # Normalize derivative
    dt /= np.linalg.norm(dt, 2)
    tangent = np.array([point, point + dt])
    normal = np.array([point, point + [- dt[1], dt[0]]])
    curvature_center = point + np.array([- dt[1], dt[0]]) * radius
    circle = plt.Circle(tuple(curvature_center), radius,
                        color=(0, 0.8, 0.8), fill=False, linewidth=1)

    assert path.T[0][0] == start_x, "path is invalid"
    assert path.T[1][0] == start_y, "path is invalid"
    assert path.T[0][-1] == end_x, "path is invalid"
    assert path.T[1][-1] == end_y, "path is invalid"

    if show_animation:  # pragma: no cover
        fig, ax = plt.subplots()
        ax.plot(path.T[0], path.T[1], label="Bezier Path")
        ax.plot(control_points.T[0], control_points.T[1],
                '--o', label="Control Points")
        ax.plot(x_target, y_target)
        ax.plot(tangent[:, 0], tangent[:, 1], label="Tangent")
        ax.plot(normal[:, 0], normal[:, 1], label="Normal")
        ax.add_artist(circle)
        plot_arrow(start_x, start_y, start_yaw)
        plot_arrow(end_x, end_y, end_yaw)
        ax.legend()
        ax.axis("equal")
        ax.grid(True)
        plt.show()


def main2():
    """Show the effect of the offset."""
    start_x = 1.22  # [m]
    start_y = 0.0  # [m]
    start_yaw = np.radians(170.0)  # [rad]

    end_x = 0.0  # [m]
    end_y = 0.0  # [m]
    end_yaw = np.radians(180.0)  # [rad]

    for offset in np.arange(1.0, 5.0, 1.0):
        path, control_points = calc_4points_bezier_path(
            start_x, start_y, start_yaw, end_x, end_y, end_yaw, offset)
        assert path.T[0][0] == start_x, "path is invalid"
        assert path.T[1][0] == start_y, "path is invalid"
        assert path.T[0][-1] == end_x, "path is invalid"
        assert path.T[1][-1] == end_y, "path is invalid"

        #if show_animation:  # pragma: no cover
            #plt.plot(path.T[0], path.T[1], label="Offset=" + str(offset))
    
    derivatives_cp = bezier_derivatives_control_points(control_points, 2)

    path_1_diff = calc_bezier_path(derivatives_cp[1])
    path_2_diff = calc_bezier_path(derivatives_cp[2])

    for i in range(0, 11, 1):
        dt = bezier(i*0.1, derivatives_cp[1])
        ddt = bezier(i*0.1, derivatives_cp[2])
        print("Curvature at ",i*0.1," is ",curvature(dt[0], dt[1], ddt[0], ddt[1]))
    #    print("Curvature at ",i*0.1," is ",1/bezier_first_deriv_poly(i*0.1,control_points)) 
    if show_animation:  # pragma: no cover
        plot_arrow(start_x, start_y, start_yaw)
        plot_arrow(end_x, end_y, end_yaw)
        plt.plot(control_points.T[0], control_points.T[1],
                '--o', label="Control Points")
        plt.plot(path.T[0], path.T[1], label="Path 1")
        #plt.plot(path_1_diff.T[0], path_1_diff.T[1], label="Path 2")
        #plt.plot(path_2_diff.T[0], path_2_diff.T[1], label="Path 3")
        plt.legend()
        plt.axis("equal")
        plt.grid(True)
        plt.show()


if __name__ == '__main__':
    #main()
    main2()
