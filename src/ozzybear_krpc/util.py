import copy
import keyword

import krpc

from hb_ksp import const

def get_conn(name):
    return krpc.connect(name=name)


def get_unfucked_expression(conn):
    """
    [conn.krpc.]Expression's methods can be python keywords, so screw around
    with Expression's __dict__ (not directly, I use setattr, I'm not a
    monster) and add an underscore ("_") to the end of the method (e.g.
    Expression.add becomes Expression.add_ ).

    Arguments:
        *conn* <krpc.client.Client> : a krpc connection [client].

    Returns:
        *Expression* <conn.krpc.Expression> : a krpc Expression class that has functional members

    Author:
        * Tyler Jachetta

    """
    Expression = conn.krpc.Expression

    if not getattr(Expression, 'unfucked', False) is True:

        for member_name, member in dict(Expression.__dict__.items()).items():
            if keyword.iskeyword(member_name):
                setattr(Expression, member_name + '_', member)

        Expression.unfucked = True

    return Expression

