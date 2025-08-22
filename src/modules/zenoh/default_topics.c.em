@###############################################
@#
@# EmPy template
@#
@###############################################
@# Generates default config fore Zenoh
@#
@# Context:
@#  - msgs (List) list of all RTPS messages
@#  - topics (List) list of all topic names
@#  - spec (msggen.MsgSpec) Parsed specification of the .msg file
@###############################################
@{
import os


}@

const char* default_pub_config =
@[    for pub in publications]@
	"@(pub['topic']);@(pub['simple_base_type']);0\n"
@[    end for]@
;

const char* default_sub_config =
@[    for sub in subscriptions]@
	"@(sub['topic']);@(sub['simple_base_type']);0\n"
@[    end for]@
@[    for sub in subscriptions_multi]@
	"@(sub['topic']);@(sub['simple_base_type']);-1\n"
@[    end for]@
;
