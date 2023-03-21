
Blue=$'\033[1;34m'
Green=$'\033[1;32m'
Red=$'\033[1;31m'
Yellow=$'\033[1;33m'
Noc=$'\033[0m'

clear


if [[ $1 == menu ]]
then 
    echo "${Green}Running: idf.py menuconfig"
    echo ${Noc}
    idf.py menuconfig
elif [[ $1 == build ]]
then 
    echo "${Yellow}Running: idf.py build"
    echo ${Noc}
    idf.py build
elif [[ $1 == run ]]
then
    echo "${Blue}Running: idf.py build"
    echo ${Noc}
    idf.py build
    echo "${Blue}Running: idf.py -p /dev/ttyUSB0 flash monitor"
    echo ${Noc}
    idf.py -p /dev/ttyUSB0 flash monitor
elif [[ $1 == clear ]]
then 
    echo "${Yellow}Running: idf.py fullclean"
    echo ${Noc}
    idf.py fullclean
elif [[ $1 == usb ]]
then 
    echo "${Green}Running: ls /dev/tty*"
    ls /dev/tty*
elif [[ $1 == chown ]]
then
    echo "${Green}Running: sudo chown nds /dev/ttyUSB0"
    sudo chown nds /dev/ttyUSB0
else
    echo "${Red}Wrong option."
    echo ${Noc}
fi



