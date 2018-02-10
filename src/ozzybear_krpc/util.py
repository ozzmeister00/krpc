
import krpc

def get_conn(name, address=None):
    return krpc.connect(name=name, address=address)


