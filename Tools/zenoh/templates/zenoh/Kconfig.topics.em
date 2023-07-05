@{
topics_count = len(topics)
topic_names_all = list(set(topics)) # set() filters duplicates
topic_names_all.sort()

datatypes = list(set(datatypes)) # set() filters duplicates
datatypes.sort()
}@

menu "Zenoh publishers/subscribers"
	depends on MODULES_ZENOH
@[for idx, topic_name in enumerate(datatypes)]@
	config ZENOH_PUBSUB_@(topic_name.upper())
		bool "@(topic_name)"
		default n

@[end for]

config ZENOH_PUBSUB_ALL_SELECTION
	bool
	default y if ZENOH_PUBSUB_ALL
@[for idx, topic_name in enumerate(datatypes)]@
	select ZENOH_PUBSUB_@(topic_name.upper())
@[end for]
endmenu
