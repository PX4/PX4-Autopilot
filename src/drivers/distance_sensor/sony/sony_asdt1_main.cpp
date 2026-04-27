#include "as_dt1.hpp"

int main()
{
	AS_DT1 as_dt1;

	// Connect to AS-DT1
	as_dt1.connect();

	// Start reading data
	as_dt1.startReading();

	// Write command to AS-DT1 to start streaming
	as_dt1.writeCommand("t");

	// Wait for some time to accumulate data
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));

	// Convert the latest read binary data to PCD format
	as_dt1.convertBinaryToPCD();

	// Stop reading data
	as_dt1.stopReading();

	return 0;
}