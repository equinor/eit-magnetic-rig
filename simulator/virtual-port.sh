#/bin/bash

# Create two virtual serial ports that can be used to simulate a serial connection between two programs.
# Create a TCP connection to the first program and forward its data stream to one of the virtual serial ports.
# The second program can then connect to the other virtual serial port and communicate with the first program.
# This script requires socat to be installed on the system.

# Do note that pty ("pseudo-teletype", where a serial port is a "real teletype") is also used by login terminals.
# Many serial port parameters, e.g. baudrate, parity, hw flow control, character size (?) are not implemented in pty. 
# It is thus impossible to test your application in presence of serial transmission errors.


printf "Check for neccesary executables... "
hash socat 2>/dev/null || {
    echo -e "\nERROR: socat not found in PATH. Exiting... " >&2
    exit 1
}
printf "OK\n"

# This creates two virtual serial ports that are connected to each other. The adresses are random.
# socat -d -d pty,raw,echo=0 pty,raw,echo=0

# Create two virtual serial ports with a specific name (symlinks) and permissions
# ttyVA00 is READ and ttyVB00 is WRITE

WORK_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PORT_READ="${WORK_DIR}/virtual-ports/READ" #"/tmp/vcom0"
PORT_WRITE="${WORK_DIR}/virtual-ports/WRITE" #
PORT_TCP=3000

# socat -d -d pty,link="${PORT_READ}",echo=0,perm=0777 pty,link="${PORT_WRITE}",echo=0,perm=0777

# socat -d -d pty,link=/tmp/vserial1,raw,echo=0 pty,link=/tmp/vserial2,raw,echo=0
printf "Virtual serial port READ: ${PORT_READ}\n"
printf "Virtual serial port WRITE: ${PORT_WRITE}\n\n"


# debug
socat -d -d pty,link=${PORT_READ},raw,echo=0 pty,link=${PORT_WRITE},raw,echo=0
exit

nohup socat -d -d pty,link=${PORT_READ},raw,echo=0 pty,link=${PORT_WRITE},raw,echo=0 > /dev/null 2>&1 &
VIRTUAL_PORTS_PID=$!

echo "Virtual ports created with PID: ${VIRTUAL_PORTS_PID}"



# Create a TCP connection and forward its data stream to the READ virtual serial port
socat TCP:localhost:3000 "${PORT_READ}",b115200,raw,echo=0

# kill ${VIRTUAL_PORTS_PID} 2>/dev/null
kill -2 "${VIRTUAL_PORTS_PID}"

echo "Closed all virtual ports."

# Now ttyVB00 is accessible with minicom, for example:
# minicom -b 115200 -o -D "{$PORT_WRITE}"