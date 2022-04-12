# Message with information for a dish of pasta
uint64 timestamp			# [us] time when the topic was published

uint16 customer_table_id		# customer’s table ID to know where to serve

uint8 menu_name				# menu, e.g. Carbonara, Amatriciana

uint8 PASTA_MENU_UNDEFINED = 0		# Undefined: Default value if no value is set
uint8 PASTA_MENU_CARBONARA = 1		# Carbonara: With Egg, Pecorino and Guanciale!
uint8 PASTA_MENU_AMATRICIANA = 2	# Amatriciana: With Tomato, Pecorino and Guanciale!
uint8 PASTA_MENU_AGLIO_E_OLIO = 3	# Aglio E Olio: With Olive oil and Garlic!
uint8 PASTA_MENU_BOLOGNESE = 4		# Bolognese: With Beef and Tomato!

uint8 cooked_texture			# how cooked the pasta should be, e.g. Al Dente

uint8 PASTA_COOKED_UNDEFINED = 0	# Undefined: Default value if no value is set
uint8 PASTA_COOKED_AL_DENTE = 1		# Al Dente: https://en.wikipedia.org/wiki/Al_dente
uint8 PASTA_COOKED_RAW = 2		# Barely cooked

uint8 pasta_type			# type of pasta, e.g. Spaghetti, Lasagne, Rigatoni

uint8 PASTA_TYPE_UNDEFINED = 0		# Undefined: Default value if no value is set
uint8 PASTA_TYPE_SPAGHETTI = 1		# Spaghetti: Long, stringy pasta
uint8 PASTA_TYPE_RIGATONI = 2		# Rigatoni: Cylindrical pasta perfect for Carbonara!
uint8 PASTA_TYPE_LASAGNE = 3		# Lasagne: Flat, big pasta that gets layered on each other

float32 pasta_temperature		# [deg C] temperature of the pasta

# TOPICS pasta_cook pasta_order

# The topic pub/sub flow would be as follows:
# Customer -> 'pasta_order' -> Waiter
# Waiter -> 'pasta_cook' -> Chef
