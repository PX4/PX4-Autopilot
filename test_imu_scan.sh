#!/bin/bash
# Script to systematically test ICM42688 on different SPI buses and CS pins

echo "Testing ICM42688 on different SPI buses and chip select pins..."
echo "This will help us find where the IMU actually is"
echo ""

# Test SPI1 with different CS pins
for cs in 1 2 3 4; do
    echo "Testing SPI1, CS=$cs"
    echo "icm42688 start -s -b 1 -c $cs"
done

# Test SPI2 with different CS pins
for cs in 1 2 3 4; do
    echo "Testing SPI2, CS=$cs"
    echo "icm42688 start -s -b 2 -c $cs"
done

# Test SPI4 with different CS pins
for cs in 1 2 3 4; do
    echo "Testing SPI4, CS=$cs"
    echo "icm42688 start -s -b 4 -c $cs"
done

echo ""
echo "After each test, run: dmesg | grep 'Found ICM'"
echo "If you see 'Found ICM42688', you've found the correct bus/CS!"

