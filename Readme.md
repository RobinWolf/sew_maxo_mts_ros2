# sew_agv_drivers

This Repo contains ROS2 code to controll a SEW AGV. The communication to the AGV is based on the Repo LinkTODO by Tom Schneider.

## Helpful informations:
ROS Control: https://control.ros.org/humble/doc/ros2_control/doc/index.html 

Diffdrive Repo by ArticulatedRobotics: https://github.com/joshnewans/diffdrive_arduino/tree/humble 


## Helpful comands
### Launch single packages
Launch driver:
```
ros2 launch sew_agv_drivers launch_agv.launch.py rviz:=true use_fake_hardware:=true
```
Move agw via terminal:
```
ros2 topic pub /diffbot_base_controller/cmd_vel geometry_msgs/msg/TwistStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, twist: {linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}}" --once
```
Move agv via joystick:
```
ros2 launch sew_agv_navigation joystick.launch.py
```
### Launch all together with gazebo or real hardware
```
ros2 launch sew_agv_navigation bringup.launch.py
```
```
ros2 launch sew_agv_navigation navigation.launch.py launch_rviz:=true
```






## Hardware Interface
### Setup connection to AGV:
```
sudo ip addr add 192.168.10.222/24 dev enx9cebe8ea463d
sudo ip addr show enx9cebe8ea463d
```

Here `enx9cebe8ea463d` is the Ethernet interface connected to your network. To find out type: `ip addr show`. Something like this shows up:
```
4: enx9cebe8ea463d: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500 qdisc fq_codel state UP group default qlen 1000
    link/ether 9c:eb:e8:ea:46:3d brd ff:ff:ff:ff:ff:ff
```
(on raspi4:)
```
sudo ip addr add 192.168.10.222/24 dev eth0
sudo ip addr show eth0
```

### Launch hardware interface
```
ros2 launch sew_agv_drivers launch_agv.launch.py 
```



## TODOs/ aktueller Stand Hardware Interface
- Joystick einbilden --> funktioniert, in cmd variable wird Wert eingetragen, an HW interface wird bisher nur dummy Wert übergeben
- connection funktioniert jetzt, wenn Ethernetanschluss entsprechend konfiguriert wird (siehe readme)
- state vom agv wird am anfang ausgelesen (ersten werte sinnvoll, hintere werte (batteriestand, ...) haben komische werte) --> funktioniert nur ein paar mal, dann nichtmehr
- dummy cmds an agv senden: keine Fehler, 72 Bytes werden gesendet, Agv reagiert nicht. Checken wie die Daten vor dem senden encodiert werden und ob das so richtig ist

01.07.2024
- AGV mit dummy x und y direct aus hardare interface write funktioniert (aber erst ab controller manager update rate von 1000)
- TODO: Mit controller und richtigen werten testen
- TODO: Status richtig lesen
- TODO: pos und vel für stateinterface richtig berechnen (oder kann man state interface weg lassen?)

07.07.2024
- open_loop = true --> odom wird aus cmd berechnet, feedback nicht notwendig --> TODO: check if odom seems to be correct
- Steuerung des AGVs mit Controller erfolgreich, stoppt manchmal, wenn zu schnell hält AGV an und zeigt Fehler "Sicherheitsfunktion SOS" an
- Status AGV wird nach wie vor nur am Anfang gelesen, danach keine Antwort mehr --> einfach in on_activate verschieben und read entfernen? Status muss eigentlich nicht die ganze zeit gelesen werden


09.07.2024
- Status vom AGV nur einmal in on_activate() lesen, nichtmehr zyklisch in read()
- TODO mit Raspi testen


24.07.2024
- Connection zum AGV wird nicht durchgängig gehalten --> extra thread, der connection aufrecht erhält
- wenn connection_ = true; in zeile 82 in agv_enpoint auskommentiert: Verbindung mit agv stabil, aber rest funktioniert natürlich nichtmehr (Wenn einkommentiert: Verbindung mit agv nicht stabil)
- TODO: Prüfen ob buffer nach senden wirklich leer ist
- TODO: Errocode AGV Status checken
- TODO: Buffer prüfen ob vector oder array


25.07.2024
- IP und Port bei msg an AGV weglassen --> Verbindung bleibt stabil
- Cahnged speedmode to CREEP --> AGV wird nichtmehr zu schnell und stoppt desswegen nichtmehr
- Speed berechnet statt 100 hardgecodet
- Status wird nur einmal am ANfang geprintet

- TODO: Alles auf raspi laufen lassen mit fernsteuerung pc
- TODO: Autostart