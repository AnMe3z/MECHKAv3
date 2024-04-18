#!/bin/bash

# Define scale for the floating point numbers
SCALE=3

# Set the broadcast address and port
BROADCAST_ADDRESS="255.255.255.255"
PORT="12345"

while true; do
# Ask for the data to be sent
echo "Enter the data to be sent:"
read DATA

# Convert the data to bytes
BYTE_DATA=$(echo -n "$DATA" | xxd -pu -c 256)

# Send the data
echo -n "$BYTE_DATA" | socat - UDP-DATAGRAM:$BROADCAST_ADDRESS:$PORT,broadcast

# Check if the data has been sent successfully
if [ $? -eq 0 ]; then
echo "Data has been sent successfully."
echo "$BYTE_DATA"
else
echo "Failed to send data."
fi
done
