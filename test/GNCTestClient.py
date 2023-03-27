from threading import Thread, Lock
import copy
import msgpack
import time
import os
import signal

DOWNLINK_FILE = "/tmp/satdownlink"
UPLINK_FILE = "/tmp/satuplink"
MAGIC_PACKET_SIZE = 43
TIME_INTERVAL = 10

class ThreadSafeState:
    
    def __init__(self, state):
        self.state = state
        self.lock = Lock()
    
    def get(self):
        with self.lock:
            return copy.copy(self.state)
    
    def set(self, state):
        with self.lock:
            self.state = state
    

class GNCTestClient:

    def __init__(self, uplink_interval=2.0, downlink_interval=1.0):
        self._state = {}
        self._prev_time = time.time()
        self._log_fd = open("/tmp/satlog.txt", "w")
        self._uplink_interval = uplink_interval
        self._downlink_interval = downlink_interval
    
    def register_state(self, key, value):
        self._state[key] = ThreadSafeState(value)
    
    def __getitem__(self, key):
        return self._state[key].get()
    
    def __setitem__(self, key, value):
        self._state[key].set(value)
    
    def downlink(self):
        read_file = open(DOWNLINK_FILE, "rb")
        data = read_file.read()
        if len(data) == 0:
            raise Exception('No downlinked data')
        data = msgpack.unpackb(data)
        for key, value in data.items():
            self[key] = value
    
    def uplink(self):
        control = { 
            "m": self["control"],
            "dt": time.time() - self._prev_time
        }
        payload = msgpack.packb(control, use_bin_type=True)

        self._prev_time = time.time()

        if len(payload) != MAGIC_PACKET_SIZE:
            raise Exception('Invalid payload size')

        self._uplink_fd.write(payload)
        self._uplink_fd.flush()
    
    def communication_thread(self):
        downlink_time = time.time()
        uplink_time = time.time()
        while True:
            if time.time() > downlink_time:
                try:
                    self.downlink()
                except Exception as e:
                    self.log(f'Error reading downlinked data:\n {e}')
                downlink_time += self._downlink_interval
            if time.time() > uplink_time:
                try:
                    self.uplink()
                except Exception as e:
                    self.log(f'Error sending uplink data:\n {e}')
                uplink_time += self._uplink_interval

            next_uplink = uplink_time - time.time()            
            next_downlink = downlink_time - time.time()

            sleep_time = min(next_uplink, next_downlink)

            if sleep_time < 0:
                self.log(f'Warning: sleep time is negative: {sleep_time}')
                os.kill(os.getpid(), signal.SIGINT)

            time.sleep(sleep_time)
    
    def launch(self):
        self._uplink_fd = open(UPLINK_FILE, "wb")
        t = Thread(target=self.communication_thread, name="communication_thread")
        t.start()
    
    def log(self, text):
        self._log_fd.write(text)
        self._log_fd.write("\n")
