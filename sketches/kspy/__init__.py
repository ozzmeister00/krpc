

def target(v, t):
    '''
    returns vector to point at target
    in vessel.orbit.body.non_rotating_reference_frame
    '''
    rf = v.orbit.body.non_rotating_reference_frame
    return v3minus(t.position(rf), v.position(rf))


def anti_target(v, t):
    '''
    returns vector to point away from target
    in vessel.orbit.body.non_rotating_reference_frame
    '''
    rf = v.orbit.body.non_rotating_reference_frame
    return v3minus(v.position(rf), t.position(rf))


def target_vplus(v, t):
    '''
    returns vector to point at + target velocity
    in vessel.orbit.body.non_rotating_reference_frame
    '''
    rf = v.orbit.body.non_rotating_reference_frame
    return v3minus(v.velocity(rf), t.velocity(rf))


def target_vminus(v, t):
    '''
    returns vector to point at  - target velocity
    in vessel.orbit.body.non_rotating_reference_frame
    '''
    rf = v.orbit.body.non_rotating_reference_frame
    return v3minus(t.velocity(rf), v.velocity(rf))