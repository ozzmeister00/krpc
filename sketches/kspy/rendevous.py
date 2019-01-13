def time_transfer(vessel, target, ut, phase_angle):
    '''
    Performs an iterative search for the next time vessel and target
    have the given relative phase_angle after ut
    '''
    print("Doing Coarse Search for Transfer Time...")
    while True:
        v_pos = orbital_progress(vessel, ut)
        t_pos = orbital_progress(target, ut)
        angle_error = math.fabs(t_pos - (v_pos - math.pi) - phase_angle)
        if angle_error < .01:
            break
        ut += 10
    ut -= 10
    # fine unbound search
    print("Doing Fine Search for Transfer Time...")
    while True:
        v_pos = orbital_progress(vessel, ut)
        t_pos = orbital_progress(target, ut)
        angle_error = math.fabs(t_pos - (v_pos - math.pi) - phase_angle)
        if angle_error < .001:
            break
        ut += 1
    return ut