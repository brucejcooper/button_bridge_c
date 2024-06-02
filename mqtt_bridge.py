import serial
import paho.mqtt.client as mqtt
from typing import List, Dict
import json
from connection_config import uart_port, mqtt_hostname, mqtt_username, mqtt_password
from select import select
import time
import re


    
    

def parse_value(v: str) -> tuple[str,str]:
    pos = v.find('=')
    if pos == -1:
        raise Exception("No = found in array of values")
    return (v[0:pos], v[pos+1:])
    
def tokens_to_values(tokens: List[str]) -> Dict[str,str]|str:
    if len(tokens) == 1 and not '=' in tokens[0]:
        return tokens[0]
    return dict([parse_value(w) for w in tokens])

entity_types = ['switch', 'light', 'text', 'button']


def dict_to_pairs(v: bytes):
    if v.startswith(b'{'):
        return ' '.join([f"{k}={str(v)}" for k,v in json.loads(v.decode('utf8')).items()])
    else:
        return v.decode('utf8')
                  
                  
class Lexer:
    KEY = 1

    def __init__(self, val: str):
        self.val = val
        self.pos = 0
        self.len = len(val)
          
    def at_end(self):
        return self.pos >= self.len
    
    def skip_whitespace(self):
        while not self.at_end() and self.peek().isspace():
            self.pos+= 1

    def peek(self):
        if self.at_end():
            raise Exception("walked past end of string while parsing")
        return self.val[self.pos]
            
    def read_identifier(self):
        self.skip_whitespace()

        start = self.pos
        while not self.at_end() and self.val[start:self.pos+1].isidentifier():
            self.pos += 1
        if start == self.pos:
            raise Exception(f"Expected token characters, but got '{self.val[start:self.pos+1]}' at {self.pos}")
        return self.val[start:self.pos]
    
    def read_value(self):
        self.skip_whitespace()
        if self.peek() == '"':
            self.pos += 1
            start = self.pos
            while self.peek() != '"':
                self.pos += 1
            tok = self.val[start:self.pos]
            self.pos += 1
            return tok
        elif self.peek().isdigit():
            start = self.pos
            while not self.at_end() and self.peek().isdigit():
                self.pos += 1
            return int(self.val[start:self.pos])
        else:
            return self.read_identifier()

    def expect(self, ch: str):
        nxt = self.peek()
        if nxt != ch:
            raise Exception(f"Expected {ch} at {self.pos}, but read {nxt}")
        self.pos +=1

    def read_values(self):
        self.skip_whitespace()
        if self.at_end():
            return None
        if self.val.find("=", self.pos) != -1:
            # Key/value pairs
            res = {}
            while not self.at_end():
                key = self.read_identifier()
                self.skip_whitespace()
                self.expect('=')
                val = self.read_value()
                self.skip_whitespace()
                res[key] = val
            return res
        else:
            return self.val[self.pos:].strip()        
                
                  

class SerialMqttBridge:
    _mqtt_client: mqtt.Client
    _serport: serial.Serial
    
    username: str
    password: str
    hostname: str
    client_id: str
    serial_port_path: str
    reconnect_interval: int # In seconds
    
    _availability_topic: str
    


    
    def __init__(self, hostname: str, username: str, password: str, serial_port_path: str, client_id: str = "btnbridge", reconnect_interval: int = 2) -> None:
        self.hostname = hostname
        self.username = username
        self.password = password
        self.serial_port_path = serial_port_path
        self.client_id = client_id
        self.reconnect_interval = reconnect_interval
        self._availability_topic = f"mechination/{self.client_id}/available"
        pass
    


    def process_log(self, line: str):
        if line != "":
            print("LOG",line)
        
    def _send_to_mqtt(self, topic: str, msg: dict|str):
        print(f"STATUS {topic}: {msg}")
        self._mqtt_client.publish(topic, json.dumps(msg) if isinstance(msg, dict) else msg, 0)

    def infer_devname(self, s: str):
        m = re.match("^([a-zA-Z_]+)(\\d+)$",s)
        if m:
            return f"{m[1].replace("_", " ").title()} {m[2]}"
        else:
            return s
            
    def process_status(self, st: str):
        lex = Lexer(st)
        
        entity_or_type = lex.read_identifier()
        
        if entity_or_type in entity_types:
            type = entity_or_type
            entity_id = lex.read_identifier()
            values = lex.read_values()
            assert isinstance(values, dict)
            
            entity_unique_id = f"{self.client_id}_{entity_id}"
            
            last_underscore = entity_id.rfind('_')
            if last_underscore != -1:
                devid = entity_id[0:last_underscore]
            else:
                devid = entity_id
            
            topic_prefix = f"mechination/{self.client_id}/{entity_id}"
            msg =  {
                "unique_id": entity_unique_id,
                "object_id": entity_id,
                "state_topic": topic_prefix,
                "command_topic": topic_prefix + "/set", 
                "availability": [
                    { 
                        "topic": self._availability_topic
                    }
                ],
                "optimistic": False,
                "qos": 0,
                "device": {
                    "identifiers": [
                        f"{self.client_id}-{devid}"
                    ],
                    "name": self.infer_devname(devid)
                }
            }
            if type == 'light':
                msg['schema'] = 'json'
            msg.update(values)
            
            self._send_to_mqtt(f"homeassistant/{type}/{self.client_id}/{entity_id}/config", json.dumps(msg))

        else:
            entity_id = entity_or_type
            values = lex.read_values()
            topic = f"mechination/{self.client_id}/{entity_id.replace(".", '/')}"
            self._send_to_mqtt(topic, values if isinstance(values, str) else json.dumps(values))


    def enumerate_devices(self):
        self._serport.write(b"\r\nenumerate\r\n")


    def on_connect(self, client, userdata, flags, reason_code, properties):
        print("connected to MQTT")
        for topic in [f"mechination/{self.client_id}/+/set", f"mechination/{self.client_id}/+/+/set", "homeassistant/status"]:
            print(f"Subscribing to {topic}")
            self._mqtt_client.subscribe(topic)
            self._mqtt_client.publish(self._availability_topic, "online", retain=True)
        
        # Request that the devices enumerate themselves    
        self.enumerate_devices()


    def on_disconnect(self, client, userdata, flags, reason_code, properties):
        print("Disconnected form MQTT:", reason_code)
        self.disconnected = True, reason_code

    def on_message(self, client, userdata, msg: mqtt.MQTTMessage):
        print(f"Got msg {msg.topic}: {msg.payload}")
        m = re.match(f"mechination/{self.client_id}/(\w+)/set", msg.topic)
        if m is not None:
            self._serport.write(f"\r\n{m[1]} {dict_to_pairs(msg.payload)}\r\n".encode('utf8'))
        elif msg.topic == "homeassistant/status" and msg.payload == b'online':
            self.enumerate_devices()
            


    
    def main(self):
        while True:
            try:
                print("Opening connection to ", uart_port)
                self._serport = serial.Serial(port=uart_port, baudrate=115200, timeout=0)
                # self._ser_reader, self._ser_writer = await serial_asyncio.open_serial_connection(loop=loop, url=uart_port, baudrate=115200)
                print("Connected to serial port")
                
                print("Opening MQTT Connection")
                
                self._mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=self.client_id)
                try:
                    self._mqtt_client.username_pw_set(self.username, self.password)
                    self._mqtt_client.will_set(self._availability_topic, "offline", retain=True)
                    
                    
                    self._mqtt_client.on_connect = self.on_connect
                    self._mqtt_client.on_message = self.on_message
                    self._mqtt_client.on_disconnect = self.on_disconnect
                    
                    
                    self._mqtt_client.connect(self.hostname, 1883, 15)
                    self.disconnected = (False, None)
                    
                    self._readbuf = b""
                    
                    while not self.disconnected[0]:                
                        mqttsock = self._mqtt_client.socket()
                        uartsock = self._serport.fileno()
                        
                        if not mqttsock or not uartsock:
                            raise Exception("One of the sockets is gone")
            
                        readable, writeable, exlist = select([mqttsock, uartsock], 
                                                             [mqttsock] if self._mqtt_client.want_write() else [], 
                                                             [], 
                                                             1)
                        
                        if len(exlist) > 0:
                            raise Exception("One of the sockets had an exception")
                        for s in readable:
                            if s == mqttsock:
                                self._mqtt_client.loop_read()
                            elif s == uartsock:
                                self._readbuf += self._serport.readline()
                                if self._readbuf.endswith(b"\n"):
                                    line = self._readbuf.strip(b"\r\n")
                                    tabpos = line.find(b"\t")
                                    if tabpos != -1:
                                        self.process_status(line[tabpos+1:].decode('utf8'))
                                        self._readbuf = line[0:tabpos]
                                    else:
                                        self.process_log(line.decode('utf8'))
                                        self._readbuf = b""
                        for s in writeable:
                            if s == mqttsock:
                                self._mqtt_client.loop_write()
                            elif s == uartsock:
                                print("Serial is writeable")
                                
                        self._mqtt_client.loop_misc()
                finally:
                    self._mqtt_client.disconnect()
                    self._serport.close()                
            except Exception as ex:
                print(f'Error {ex} when reading from USB Serial port or MQTT.  Will try again')
                time.sleep(self.reconnect_interval)




SerialMqttBridge(mqtt_hostname, mqtt_username, mqtt_password, uart_port).main()