import krpc

def getConnection():
    connection = krpc.connect()
    # connection = krpc.connect(address='192.168.1.64',
    #                            stream_port=50001,
    #                            rpc_port=50000)

    return connection