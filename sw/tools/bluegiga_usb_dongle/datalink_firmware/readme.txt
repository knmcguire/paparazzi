
# NOTE: TX power is a signed 8-bit value, and is not changed automatically
# when using "hardware_set_txpower(<power>)". This may be anywhere from -23
# to +3 based on your settings. Negative values may be converted to two's
# complement form by adding 256, so for example -23 dBm would be 233 or $E9.

# NOTE: manufacturer data fields should contain the Company Identifier Code
# in order to stay within BT 4.0 spec. You should also ideally obtain an
# an official Company Identifier Code, but 0xFFFF is suitable for development.
# (this is why the minimum ad field length for this is 3, so the CIC fits)
# More bytes are possible here, but not necessary for this demo.
# Compare with automatically generated ad packets from other
# demo projects to see what else you might put here, or read
# the relevant portions of the Bluetooth 4.0 Core Spec document
# for greater detail.

