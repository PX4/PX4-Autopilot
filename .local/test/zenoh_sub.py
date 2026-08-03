import zenoh
from msgs.NavputAttitude import NavputAttitude
from msgs.NavputLocalPosition import NavputLocalPosition
from msgs.NavputStatusFlags import NavputStatusFlags
from msgs.NavputFusionControl import NavputFusionControl


conf = zenoh.Config()
conf.insert_json5("mode", '"client"')
conf.insert_json5("connect/endpoints", '["tcp/127.0.0.1:7447"]')

session = zenoh.open(conf)

routers = session.info.routers_zid()
print("connected routers:", routers)
if not routers:
    print("WARNING: not connected to any zenohd router")


def full_topic(topic):
    domain_id = 0
    return f"{domain_id}/fmu/out/{topic}/**"


def on_attitude(sample):
    print(NavputAttitude.deserialize(bytes(sample.payload)))

sub_local_pos = session.declare_subscriber(
    full_topic('navput_local_position'),
    lambda sample: print(NavputLocalPosition.deserialize(bytes(sample.payload)))
)
# sub_att = session.declare_subscriber(full_topic('navput_attitude'), on_attitude)
sub_status_flags = session.declare_subscriber(
    full_topic('navput_status_flags'),
    lambda sample: print(NavputStatusFlags.deserialize(bytes(sample.payload)))
)
sub_fusion_control = session.declare_subscriber(
    full_topic('navput_fusion_control'),
    lambda sample: print(NavputFusionControl.deserialize(bytes(sample.payload)))
)


import time
while True:
    time.sleep(1)
