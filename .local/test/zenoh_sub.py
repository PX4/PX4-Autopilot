import zenoh
from msgs.SensorCombined import SensorCombined

conf = zenoh.Config()
conf.insert_json5("mode", '"client"')
conf.insert_json5("connect/endpoints", '["tcp/127.0.0.1:7447"]')

session = zenoh.open(conf)

routers = session.info.routers_zid()
print("connected routers:", routers)
if not routers:
    print("WARNING: not connected to any zenohd router")

domain_id = 0
topic = "/fmu/out/sensor_combined"
keyexpr = f"{domain_id}{topic}/**"  # ** skips the px4_msgs/RIHS01 hash suffix

def listener(sample):
    print(sample.key_expr, "->", SensorCombined.deserialize(bytes(sample.payload)))

sub = session.declare_subscriber(keyexpr, listener)

import time
while True:
    time.sleep(1)
