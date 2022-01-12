from casadi.casadi import fmax, fmin


def cs2bool(residual, with_else=False):
    '''
    Function to convert if-statement into 0 or 1 with casadi syntax.
    if-statement has to be in the form of:
        if residual > 0:
            ...
        else:
            ...
    '''
    # return if residual > 0 -> 1 and if residual <= 0 -> 0
    # if max is zero -> is smaller equal to 1 -> get 0
    # if max is nonzero -> max = residual -> (max + residual)*max = 2*residual**2 -> 2*residual**2/residual**2=2 -> 1<2 -> get 1
    
    max = fmax(0.0, residual)
    boolean = fmin(1.0,(max+residual)*max/(residual**2+1e-7))  # divided by residual**2 since we want to get 1 if max is non zero. For the case of residual = 0 we add a constant for numerical stability 
    nboolean = (boolean-1)**boolean   # negate boolean: 0 turns into 1, 1 turns into 0 -> if b is true then nb is not true and vice versa
    if with_else:
        return [boolean,nboolean]
    else:
        return boolean
