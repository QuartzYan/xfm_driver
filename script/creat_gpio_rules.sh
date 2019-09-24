echo "${green}Create a new gpio user group. Then add your user to the newly created group.${reset}"
sudo groupadd -f -r gpio
sudo usermod -a -G gpio $USER

echo "${green}Create a new gpio user group. Then add your user to the newly created group.${reset}"
sudo cp ./99-gpio.rules /etc/udev/rules.d/

echo "${green}reload the udev rules${reset}"
sudo udevadm control --reload-rules && sudo udevadm trigger

