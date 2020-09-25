echo "Setting up Visual Screen Forwarding on Mac so that you can see Visualizations from within Docker"
IP=$(ifconfig en0 | grep inet | awk '$1=="inet" {print $2}')
echo "Your IP that was found is:"
echo $IP
xhost + $IP
echo "Complete"