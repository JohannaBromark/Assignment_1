
import ctypes
import vrep as remote_api
from threading import Lock
import time
import pypot

# Dictopnary with the streaming modes
vrep_mode = {
    'normal': remote_api.simx_opmode_oneshot_wait,
    'streaming': remote_api.simx_opmode_streaming,
    'sending': remote_api.simx_opmode_oneshot,
}

# Dictionary with the different errors
vrep_error = {
    remote_api.simx_return_ok: 'Ok',
    remote_api.simx_return_novalue_flag: 'No value',
    remote_api.simx_return_timeout_flag: 'Timeout',
    remote_api.simx_return_illegal_opmode_flag: 'Opmode error',
    remote_api.simx_return_remote_error_flag: 'Remote error',
    remote_api.simx_return_split_progress_flag: 'Progress error',
    remote_api.simx_return_local_error_flag: 'Local error',
    remote_api.simx_return_initialize_error_flag: 'Init error'
}

#Dictionary for all the object handles in the scene
object_handles = {}

MAX_ITER = 5
lock = Lock()
# clientID should be from opening the connection
CLIENTID = 1
TIMEOUT = 0.4



def add_cube(name, position, sizes, mass):
    """ Add Cube """
    create_pure_shape(0, 239, sizes, mass, [0, 0])
    set_object_position("Cuboid", position)
    change_object_name("Cuboid", name)


def create_pure_shape(primitive_type, options, sizes, mass, precision):
    """ Create Pure Shape"""
    lua_code = "simCreatePureShape({}, {}, {{{}, {}, {}}}, {}, {{{}, {}}})".format(
        primitive_type, options, sizes[0], sizes[1], sizes[2], mass, precision[0], precision[1])
    print(lua_code)

    inject_lua_code(lua_code)

def inject_lua_code(lua_code):
    """ Sends raw lua code and evaluate it without any checking! """
    msg = (ctypes.c_ubyte * len(lua_code)).from_buffer_copy(lua_code.encode())
    call_remote_api('simxWriteStringStream', 'my_lua_code', msg)


def set_object_position(object_name, position=[0, 0, 0]):
    """ Sets the object position. """
    h = get_object_handle(object_name)
    print("handle: ")
    print(h)
    return call_remote_api('simxSetObjectPosition',
                                h, -1, position,
                                sending=True)

def change_object_name(old_name, new_name):
    """ Change object name """

    h = get_object_handle(old_name)
    if old_name in object_handles:
        object_handles.pop(old_name)
    lua_code = "simSetObjectName({}, '{}')".format(h, new_name)
    inject_lua_code(lua_code)


def get_object_handle(obj):
    """ Gets the vrep object handle. """
    if obj not in object_handles:
        print(str(obj)+" finns inte i dict")
        object_handles[obj] = vrep_get_object_handle(obj=obj)

    return object_handles[obj]


def vrep_get_object_handle(obj):
    return call_remote_api('simxGetObjectHandle', obj)


def call_remote_api(func_name, *args, **kwargs):
    """ Calls any remote API func in a thread_safe way.

    :param str func_name: name of the remote API func to call
    :param args: args to pass to the remote API call
    :param kwargs: args to pass to the remote API call

    .. note:: You can add an extra keyword to specify if you want to use the streaming or sending mode. The oneshot_wait mode is used by default (see `here <http://www.coppeliarobotics.com/helpFiles/en/remoteApiConstants.htm#operationModes>`_ for details about possible modes).

    .. warning:: You should not pass the clientId and the operationMode as arguments. They will be automatically added.

    As an example you can retrieve all joints name using the following call::

        vrep_io.remote_api_call('simxGetObjectGroupData',
                                vrep_io.remote_api.sim_object_joint_type,
                                0,
                                streaming=True)
    """

    # Retrieves the given function name
    f = getattr(remote_api, func_name)

    mode = extract_mode(kwargs)
    kwargs['operationMode'] = vrep_mode[mode]
    # hard_retry = True

    if '_force' in kwargs:
        del kwargs['_force']
        _force = True
    else:
        _force = False

    for _ in range(MAX_ITER):
        with lock:
            ret = f(CLIENTID, *args, **kwargs)

        if _force:
            return

        if mode == 'sending' or isinstance(ret, int):
            err, res = ret, None
        else:
            err, res = ret[0], ret[1:]
            res = res[0] if len(res) == 1 else res

        err = [bool((err >> i) & 1) for i in range(len(vrep_error))]

        if remote_api.simx_return_novalue_flag not in err:
            break

        time.sleep(TIMEOUT)

    # if any(err) and hard_retry:
    #     print "HARD RETRY"
    # self.stop_simulation() #nope
    #
    #     notconnected = True
    #     while notconnected:
    #         self.close()
    #         close_all_connections()
    #         time.sleep(0.5)
    #         try:
    #             self.open_io()
    #             notconnected = False
    #         except:
    #             print 'CONNECTION ERROR'
    #             pass
    #
    #     self.start_simulation()
    #
    #     with self._lock:
    #         ret = f(self.client_id, *args, **kwargs)
    #
    #         if mode == 'sending' or isinstance(ret, int):
    #             err, res = ret, None
    #         else:
    #             err, res = ret[0], ret[1:]
    #             res = res[0] if len(res) == 1 else res
    #
    #         err = [bool((err >> i) & 1) for i in range(len(vrep_error))]
    #
    #         return res

    #if any(err):
    #    msg = ' '.join([vrep_error[2 ** i]
    #                    for i, e in enumerate(err) if e])
    #    raise VrepIOErrors(msg)

    #return res


def extract_mode(kwargs):
    """Returns the mode sent to v-rep (I think)"""
    for mode in ('streaming', 'sending'):
        if mode in kwargs:
            kwargs.pop(mode)
            return mode
    return 'normal'



# V-Rep errors

class VrepIOError(Exception):
    """ Base class for V-REP IO Errors. """

    def __init__(self, error_code, message):
        message = 'V-REP error code {} ({}): "{}"'.format(
            error_code, vrep_error[error_code], message)
        Exception.__init__(self, message)

class VrepIOErrors(Exception):
    pass



class VrepConnectionError(Exception):
    """ Base class for V-REP connection Errors. """
    pass



remote_api.simxFinish(-1)
# Connect to the V-REP continuous server
CLIENTID = remote_api.simxStart('127.0.0.1', 19997, True, True, 500, 5)

if CLIENTID != -1:  # if we connected successfully
    print('Connected to remote API server')

remote_api.simxSynchronous(CLIENTID, True)



dt = 0.1
remote_api.simxSetFloatingParameter(CLIENTID,
                              remote_api.sim_floatparam_simulation_time_step,
                              dt,  # specify a simulation time step
                              remote_api.simx_opmode_oneshot)

remote_api.simxStartSimulation(CLIENTID, remote_api.simx_opmode_blocking)

add_cube("Kub", [0, 0, 0.025], [1, 1, 1], 5)
remote_api.simxStopSimulation(CLIENTID,
                        remote_api.simx_opmode_blocking)

# Now close the connection to V-REP:
remote_api.simxFinish(CLIENTID)