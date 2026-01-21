#!/bin/bash
source /opt/ros/humble/setup.bash

# Check build
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
else
    echo "Warning: install/setup.bash not found!"
fi

# Fix Line Endings for specific files (including this one if needed/possible)
# We handle this one's own EOL by processing it before execution or assuming LF
find src -name "*.py" -exec sed -i "s/\r$//" {} +
sed -i "s/\r$//" baslat.sh
chmod +x baslat.sh

# Start Menu
./baslat.sh