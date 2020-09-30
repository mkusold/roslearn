echo "This program sets up your mac. It only needs to be run once"

echo "Setting up Visual Screen Forwarding on Mac so that you can see XQuartz Visualizations from within Docker"
IP=$(ifconfig en0 | grep inet | awk '$1=="inet" {print $2}')
xhost + $IP
echo "Your IP is $IP. Copy this into the .env file"

echo "Setting up Audio Connections"
brew install pulseaudio
echo "Pulseaudio will start automatically from now on"
brew services start pulseaudio
PULSE_SETTING_LOCATION=$(mdfind -onlyin /usr/local/Cellar/pulseaudio/**/etc  kind:folder "pulse")
cp "$PULSE_SETTING_LOCATION/default.pa" "$PULSE_SETTING_LOCATION/default-orig.pa"

sed -i'.original' -e '/^#.*module-native-protocol-tcp/s/^#//' "$PULSE_SETTING_LOCATION/default.pa"
sed -i'.original' -e  '/^#.*module-esound-protocol-tcp/s/^#//' "$PULSE_SETTING_LOCATION/default.pa"
echo "Complete"